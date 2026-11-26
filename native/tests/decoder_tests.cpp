// kp_decoder_tests: the decoder's sync tracking on a signal made here (ctest: "decoder"). Plain checks, as in
// radio_tests.cpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <vector>

#include "video.h"

using namespace kp;

static int g_checks = 0, g_failed = 0;
#define CHECK(c)                                                                  \
  do {                                                                            \
    g_checks++;                                                                   \
    if (!(c)) {                                                                   \
      g_failed++;                                                                 \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c);  \
    }                                                                             \
  } while (0)

// ---------------------------------------------------------------------------------------- the signal
// A grey NTSC picture as the decoder gets it: discriminator output in MHz at 20 MS/s, sync at the low end.
// Interlaced, with the vertical interval's equalising and broad pulses; no colour, no noise.
static constexpr double kFs = 20e6, kUs = kFs * 1e-6, kT = 63.5556 * kUs;  // samples
static constexpr float kTip = -2.f, kBlank = 0.f, kGrey = 1.5f;
static constexpr int kHalfLinesPerField = 525;

struct Signal {
  std::vector<float> d;
  // [from, to) in samples, as far as it lies inside the signal
  void put(double from, double to, float level) {
    const int64_t a = std::max<int64_t>(0, std::llround(from));
    const int64_t b = std::min<int64_t>(static_cast<int64_t>(d.size()), std::llround(to));
    for (int64_t i = a; i < b; i++) d[i] = level;
  }
};

// seconds of it. late(line): how many samples after its place on the grid that line (counted from the start)
// begins, for a transmitter whose line timing steps. damage(signal, line, start): called for every picture line
// once the signal is complete, with the sample its hsync begins at.
static Signal video(double seconds, const std::function<double(int64_t)> &late,
                    const std::function<void(Signal &, int64_t, double)> &damage) {
  Signal s;
  s.d.assign(static_cast<size_t>(seconds * kFs), kBlank);
  const int64_t half_lines = static_cast<int64_t>(s.d.size() / (kT / 2)) + 2;
  const auto at = [&](int64_t g) { return g * (kT / 2) + late(g / 2); };
  const auto vertical = [](int64_t g) { return g % kHalfLinesPerField < 18; };
  for (int64_t g = 0; g < half_lines; g += 2)  // the picture of every line
    s.put(at(g) + 9.4 * kUs, at(g) + 62.0 * kUs, kGrey);
  for (int64_t g = 0; g < half_lines; g++) {  // the pulses
    const double t = at(g);
    if (vertical(g)) {
      const int in = static_cast<int>(g % kHalfLinesPerField);
      s.put(t, t + kT / 2, kBlank);
      s.put(t, t + (in >= 6 && in < 12 ? kT / 2 - 4.7 * kUs : 2.3 * kUs), kTip);
    } else if (g % 2 == 0) {
      s.put(t, t + 4.7 * kUs, kTip);
    }
  }
  if (damage)
    for (int64_t g = 0; g < half_lines; g += 2)
      if (!vertical(g) && !vertical(g + 1) && !vertical(g - 1)) damage(s, g / 2, at(g));
  return s;
}

static double on_the_grid(int64_t) { return 0; }

static DecoderStats decode(const Signal &s) {
  FrontEnd fe(kFs, 0, [](const Frame &) {});
  const std::vector<uint8_t> no_marks;
  std::vector<float> block;
  for (size_t at = 0; at < s.d.size(); at += 4096) {
    block.assign(s.d.begin() + at, s.d.begin() + std::min(s.d.size(), at + 4096));
    fe.decode(block, no_marks);
  }
  return fe.stats();
}

// ---------------------------------------------------------------------------------------- the checks

static const DecoderStats &clean() {
  static const DecoderStats st = decode(video(0.4, on_the_grid, nullptr));
  return st;
}

static void locks_on_a_clean_signal() {
  const DecoderStats &st = clean();
  CHECK(!st.acquiring && st.acquisitions == 1 && st.losses == 0);
  CHECK(st.rephases == 0);
  CHECK(st.lock_recent > 0.95f);
  CHECK(std::fabs(st.line_us - 63.5556) < 0.01);
  CHECK(std::fabs(st.tip - kTip) < 0.1f && std::fabs(st.blank - kBlank) < 0.1f);
}

// A dip that runs into the hsync from the front (the black edge of an OSD character at the end of a line, a
// click): the pulse begins 0.85 us early and is that much longer. The grid stays where it is: no line is lost,
// and nothing re-phases.
static void a_dip_ahead_of_the_hsync() {
  const DecoderStats st = decode(video(0.4, on_the_grid, [](Signal &s, int64_t line, double start) {
    if (line % 97 == 40) s.put(start - 0.85 * kUs, start, kTip - 1.f);
  }));
  CHECK(st.rephases == 0);
  CHECK(st.acquisitions == 1 && st.losses == 0);
  CHECK(st.lines_locked == clean().lines_locked);
}

// One that splits the hsync: 1.3 us of it, a gap, and the rest, which alone is long enough to be an hsync and
// begins 1.6 us late.
static void a_gap_in_the_hsync() {
  const DecoderStats st = decode(video(0.4, on_the_grid, [](Signal &s, int64_t line, double start) {
    if (line % 97 == 40) s.put(start + 1.3 * kUs, start + 1.6 * kUs, kBlank + 1.f);
  }));
  CHECK(st.rephases == 0);
  CHECK(st.lines_locked == clean().lines_locked);
}

// The transmitter's line timing steps: every line from one on comes later. Inside the window (5.1 us either
// side of where the hsync is due) the line PLL follows without losing a line; past it the grid re-phases, once.
static void a_step_in_the_line_timing() {
  for (const double us : {0.5, 1.5, 4.0, -0.5, -4.0}) {
    const DecoderStats st = decode(video(0.4, [us](int64_t line) { return line >= 3000 ? us * kUs : 0.0; }, nullptr));
    CHECK(st.rephases == 0);
    CHECK(st.lines_locked == clean().lines_locked);
    if (st.rephases || st.lines_locked != clean().lines_locked)
      std::fprintf(stderr, "  step %+.1f us: %llu re-phases, %llu lines locked (clean: %llu)\n", us,
                   (unsigned long long)st.rephases, (unsigned long long)st.lines_locked,
                   (unsigned long long)clean().lines_locked);
  }
  const DecoderStats st = decode(video(0.4, [](int64_t line) { return line >= 3000 ? 12.0 * kUs : 0.0; }, nullptr));
  CHECK(st.rephases == 1);
  CHECK(st.acquisitions == 1 && st.losses == 0);
}

// ---------------------------------------------------------------------------------------- strips

// A picture that differs from line to line, across each line, and from one field to the next (diagonal stripes
// moving down), so that a line out of place or a frame's line from the wrong field shows.
static Signal moving_stripes(double seconds) {
  return video(seconds, on_the_grid, [](Signal &s, int64_t line, double start) {
    for (double us = 9.4; us < 62.0; us += 0.05) {
      const double ph = us * 0.9 + static_cast<double>(line) * 0.71;
      s.put(start + us * kUs, start + (us + 0.05) * kUs, static_cast<float>(0.3 + 4.0 * (0.5 + 0.5 * std::sin(ph))));
    }
  });
}

struct Picture {
  std::vector<uint8_t> px;
  LineTimes times;  // of the whole picture: its first line drawn, and its last
};

constexpr double kT0 = 1000.0;  // when sample 0 was taken, as the decoder is told (sample i at kT0 + i / kFs)

// The signal through a decoder with the switches p, in blocks of 4096, each with the time its last sample was
// taken: its finished pictures. Checks as it goes that a picture's frames (several with strips) run down the picture
// without gaps or overlaps, and come out a strip's worth at a time as the lines are decoded.
static std::vector<Picture> frames_of(const Signal &s, PictureOpts p, int *strips_per_frame = nullptr) {
  std::vector<Picture> out;
  int next_y = 0, strips = 0, most = 0;
  double t_block = 0;  // when the first sample of the block being decoded was taken
  double t_top = 0;    // when the top of the picture in hand was
  bool order_ok = true, early_ok = true;
  FrontEnd fe(kFs, 0, [&](const Frame &f) {
    order_ok &= f.y0 == next_y && f.y1 > f.y0 && f.y1 <= f.height && (p.strips || f.finished());
    next_y = f.finished() ? 0 : f.y1;
    if (f.y0 == 0) t_top = f.times.t_first;
    // drawn in order; out within a few lines of the newest of them being drawn (sparkle repair holds the last 3
    // lines back, a field's last lines wait for the field to end)
    early_ok &= f.times.t_first >= kT0 && f.times.t_first <= f.times.t_last && f.times.t_last + 10 * kT / kFs > t_block;
    strips++;
    if (!f.finished()) return;
    const size_t bytes = static_cast<size_t>(f.width) * f.height * 4;
    out.push_back(Picture{std::vector<uint8_t>(f.data, f.data + bytes), LineTimes{t_top, f.times.t_last}});
    most = std::max(most, strips);
    strips = 0;
  }, DecoderOpts{}, p);
  const std::vector<uint8_t> no_marks;
  std::vector<float> block;
  for (size_t at = 0; at < s.d.size(); at += 4096) {
    block.assign(s.d.begin() + at, s.d.begin() + std::min(s.d.size(), at + 4096));
    t_block = kT0 + static_cast<double>(at) / kFs;
    fe.decode(block, no_marks, kT0 + static_cast<double>(at + block.size() - 1) / kFs);
  }
  CHECK(order_ok);
  CHECK(early_ok);
  if (strips_per_frame) *strips_per_frame = most;
  return out;
}

// With strips on, the pictures are the same as without them, lines and times, and every field goes out in strips
// that add up to it: whatever smooth lines, sparkle repair and the 3-line comb (which draws each line a line late)
// do.
static void strips_add_up_to_the_frames() {
  const Signal s = moving_stripes(0.3);
  for (int combo = 0; combo < 8; combo++) {
    PictureOpts p;
    p.smooth_lines = combo & 1;
    p.repair = combo & 2;
    p.chroma_comb = combo & 4;
    const std::vector<Picture> plain = frames_of(s, p);
    p.strips = true;
    int per_frame = 0;
    const std::vector<Picture> in_strips = frames_of(s, p, &per_frame);
    CHECK(plain.size() >= 12 && plain.size() == in_strips.size());
    bool same = plain.size() == in_strips.size(), same_times = same;
    for (size_t i = 0; same && i < plain.size(); i++) {
      same = plain[i].px == in_strips[i].px;
      const LineTimes &a = plain[i].times, &b = in_strips[i].times;
      same_times &= a.t_first == b.t_first && a.t_last == b.t_last;
    }
    CHECK(same);
    CHECK(same_times);
    // NTSC: 240 field lines, 32 to a strip (a strip may wait 3 lines for sparkle repair, one for smooth lines)
    CHECK(per_frame >= 7 && per_frame <= 9);
    if (!same || per_frame < 7 || per_frame > 9)
      std::fprintf(stderr, "  smooth %d repair %d comb %d: %zu / %zu frames, %d strips a frame\n", p.smooth_lines,
                   p.repair, p.chroma_comb, plain.size(), in_strips.size(), per_frame);
  }
}

// A frame's times: its first line was taken about a field (262.5 lines) before its last, and after the frame
// before's last. A decoder that is not told when its samples were taken says nothing.
static void frames_say_when_their_lines_were_taken() {
  const Signal s = moving_stripes(0.3);
  const std::vector<Picture> f = frames_of(s, PictureOpts{});
  CHECK(f.size() >= 8);
  for (size_t i = 2; i < f.size(); i++) {
    const double lines = (f[i].times.t_last - f[i].times.t_first) * kFs / kT;
    CHECK(lines > 235 && lines < 245);  // first row to last row: 239 lines apart
    if (i > 2) CHECK(f[i].times.t_first > f[i - 1].times.t_last);
  }
  int frames = 0, timed = 0;
  FrontEnd fe(kFs, 0, [&](const Frame &fr) {
    frames++;
    timed += fr.times.t_first != 0 || fr.times.t_last != 0;
  });
  fe.decode(s.d, {});
  CHECK(frames >= 8 && timed == 0);
}

// ---------------------------------------------------------------------------------------- level jumps

// The whole signal, sync and picture alike, moved by by(seconds) MHz. A transmitter whose video input is AC-coupled
// does that when the picture's brightness changes: seen from a camera pointed at a light flashing at 7 Hz, which
// moved the sync by 1.3 times its depth with every flash (a Pixel 8 Pro's log, 2026-11-25).
static Signal moved(Signal s, const std::function<double(double)> &by) {
  for (size_t i = 0; i < s.d.size(); i++) s.d[i] += static_cast<float>(by(static_cast<double>(i) / kFs));
  return s;
}

// by() for a light flashing at 7 Hz: the signal down by mhz while it is lit (the second half of each flash), and
// each move taking tau seconds (an RC coupling's exponential; 0: at once).
static std::function<double(double)> flashing(double mhz, double tau) {
  return [=](double t) {
    const double ph = t * 7 - std::floor(t * 7);  // 0..1 through a flash
    if (t * 7 < 0.5) return 0.0;                  // not lit before the first flash
    const bool lit = ph >= 0.5;
    const double since = (lit ? ph - 0.5 : ph) / 7, to = lit ? -mhz : 0, from = lit ? 0 : -mhz;
    return tau > 0 ? to + (from - to) * std::exp(-since / tau) : to;
  };
}

static constexpr int kJumpsASecond = 13;  // at 7 Hz, the first after half a flash

// The lock holds through them, whichever way the signal moves, by less or more than the sync depth, at once or
// over a few milliseconds: the line grid stays, no field is started that is none, and all but a few lines at each
// jump find their hsync. (A jump that lands on the lines of a vertical sync can still cost that one field: they
// have no hsync to read the new levels from, and are over before the lines after them have said.)
static void the_levels_jump_with_a_flashing_light() {
  const Signal plain = video(1.0, on_the_grid, nullptr);
  const DecoderStats steady = decode(plain);
  CHECK(steady.acquisitions == 1 && steady.losses == 0);
  for (const double depths : {0.6, 1.3, 2.0, -0.6, -1.3, -2.0})
    for (const double tau : {0.0, 0.5e-3, 2e-3, 5e-3}) {
      const DecoderStats st = decode(moved(plain, flashing(depths * (kBlank - kTip), tau)));
      const long long missed = static_cast<long long>(steady.lines_locked) - static_cast<long long>(st.lines_locked);
      const bool ok = st.acquisitions == 1 && st.losses == 0 && st.rephases == 0 && st.fields <= steady.fields &&
                      st.fields + 1 >= steady.fields && missed <= kJumpsASecond * 8 &&
                      (tau > 0 || st.level_jumps == kJumpsASecond);  // (a slow move is partly followed without)
      CHECK(ok);
      if (!ok || std::getenv("KP_TEST_VERBOSE"))
        std::fprintf(stderr, "  %+.1f depths over %.1f ms: %llu acquisitions, %llu losses, %llu re-phases, %llu fields "
                     "(steady: %llu), %lld lines without their hsync, %llu level jumps\n", depths, tau * 1e3,
                     (unsigned long long)st.acquisitions, (unsigned long long)st.losses,
                     (unsigned long long)st.rephases, (unsigned long long)st.fields,
                     (unsigned long long)steady.fields, missed, (unsigned long long)st.level_jumps);
    }
}

// And the picture keeps its brightness: its grey is drawn against the levels as they are after the jump, the lines
// that showed the jump against their own.
static void the_picture_keeps_its_grey_through_a_jump() {
  const Signal plain = video(1.0, on_the_grid, nullptr);
  const auto mid = [](const Picture &p, int row) { return p.px.data() + (static_cast<size_t>(row) * 720 + 360) * 4; };
  const std::vector<Picture> steady = frames_of(plain, PictureOpts{});
  CHECK(steady.size() >= 50);
  if (steady.size() < 50) return;
  const uint8_t grey[3] = {mid(steady[20], 240)[0], mid(steady[20], 240)[1], mid(steady[20], 240)[2]};
  const auto off_rows = [&](const std::vector<Picture> &f) {
    int off = 0;
    for (size_t i = 4; i < f.size(); i++)
      for (int row = 16; row < 464; row++)
        for (int c = 0; c < 3; c++)
          if (std::abs(mid(f[i], row)[c] - grey[c]) > 6) { off++; break; }
    return off;
  };
  CHECK(off_rows(steady) == 0);
  for (const double depths : {1.3, -1.3}) {
    const std::vector<Picture> f = frames_of(moved(plain, flashing(depths * (kBlank - kTip), 0)), PictureOpts{});
    const int off = off_rows(f);
    // the line the jump is in is off from there on; a field's line is two of the frame's, and smooth lines spread
    // it over one more either side
    const bool ok = f.size() == steady.size() && off <= kJumpsASecond * 4;
    CHECK(ok);
    if (!ok || std::getenv("KP_TEST_VERBOSE"))
      std::fprintf(stderr, "  %+.1f depths: %zu frames (steady: %zu), %d rows off their grey\n", depths, f.size(),
                          steady.size(), off);
  }
}

int main() {
  locks_on_a_clean_signal();
  a_dip_ahead_of_the_hsync();
  a_gap_in_the_hsync();
  a_step_in_the_line_timing();
  strips_add_up_to_the_frames();
  frames_say_when_their_lines_were_taken();
  the_levels_jump_with_a_flashing_light();
  the_picture_keeps_its_grey_through_a_jump();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
