// kp_decoder_tests: the decoder's sync tracking on a signal made here (ctest: "decoder"). Plain checks, as in
// radio_tests.cpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <optional>
#include <vector>

#include "field_rhythm.h"
#include "fold.h"
#include "log.h"
#include "sync_noise.h"
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

// A case's numbers are printed when it failed, or for all cases with KP_TEST_VERBOSE set.
static bool shown(bool ok) { return !ok || std::getenv("KP_TEST_VERBOSE"); }

// Noise that is the same from run to run: numbers spread evenly over [0, 1) (xorshift), and pairs of normal ones
// with the standard deviation sd (Box-Muller).
struct Noise {
  uint64_t x;
  double uniform() {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return static_cast<double>(x >> 11) * (1.0 / 9007199254740992.0);
  }
  std::pair<double, double> normal2(double sd) {
    const double r = sd * std::sqrt(-2 * std::log(1 - uniform())), ph = 2 * M_PI * uniform();
    return {r * std::cos(ph), r * std::sin(ph)};
  }
};

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
// per_field: half lines in a field (525: interlaced, each field half a line off the one before; 524: all alike).
static Signal video(double seconds, const std::function<double(int64_t)> &late,
                    const std::function<void(Signal &, int64_t, double)> &damage, int per_field = kHalfLinesPerField) {
  Signal s;
  s.d.assign(static_cast<size_t>(seconds * kFs), kBlank);
  const int64_t half_lines = static_cast<int64_t>(s.d.size() / (kT / 2)) + 2;
  const auto at = [&](int64_t g) { return g * (kT / 2) + late(g / 2); };
  const auto vertical = [per_field](int64_t g) { return g % per_field < 18; };
  for (int64_t g = 0; g < half_lines; g += 2)  // the picture of every line
    s.put(at(g) + 9.4 * kUs, at(g) + 62.0 * kUs, kGrey);
  for (int64_t g = 0; g < half_lines; g++) {  // the pulses
    const double t = at(g);
    if (vertical(g)) {
      const int in = static_cast<int>(g % per_field);
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

static DecoderStats decode(const Signal &s, const DecoderOpts &o = DecoderOpts{}) {
  FrontEnd fe(kFs, 0, [](const Frame &) {}, o);
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
static Signal moving_stripes(double seconds, int per_field = kHalfLinesPerField) {
  return video(seconds, on_the_grid, [](Signal &s, int64_t line, double start) {
    for (double us = 9.4; us < 62.0; us += 0.05) {
      const double ph = us * 0.9 + static_cast<double>(line) * 0.71;
      s.put(start + us * kUs, start + (us + 0.05) * kUs, static_cast<float>(0.3 + 4.0 * (0.5 + 0.5 * std::sin(ph))));
    }
  }, per_field);
}

struct Picture {
  std::vector<uint8_t> px;
  LineTimes times;  // of the whole picture: its first line drawn, and its last
};

constexpr double kT0 = 1000.0;  // when sample 0 was taken, as the decoder is told (sample i at kT0 + i / kFs)

// The signal through a decoder with the switches p, in blocks of 4096, each with the time its last sample was
// taken: its finished pictures. Checks as it goes that a picture's frames (several with strips) run down the picture
// without gaps or overlaps, and come out a strip's worth at a time as the lines are decoded.
// (late: how many lines after its newest line a frame may come out. kLateCoasted: where a field's vertical sync
// does not show, the field is waited on for two lines more, and a start that is found waits a line itself.)
static constexpr double kLateCoasted = 13;
static std::vector<Picture> frames_of(const Signal &s, PictureOpts p, int *strips_per_frame = nullptr,
                                      const DecoderOpts &o = DecoderOpts{}, double late = 10) {
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
    early_ok &= f.times.t_first >= kT0 && f.times.t_first <= f.times.t_last && f.times.t_last + late * kT / kFs > t_block;
    strips++;
    if (!f.finished()) return;
    const size_t bytes = static_cast<size_t>(f.width) * f.height * 4;
    out.push_back(Picture{std::vector<uint8_t>(f.data, f.data + bytes), LineTimes{t_top, f.times.t_last}});
    most = std::max(most, strips);
    strips = 0;
  }, o, p);
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
    // NTSC: 242 field lines, 32 to a strip (a strip may wait 3 lines for sparkle repair, one for smooth lines)
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
    CHECK(lines > 235 && lines < 245);  // first row to last row: 241 lines apart
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
// jump find their hsync. (A jump that lands on the lines of a vertical sync hides it: they have no hsync to read
// the new levels from, and are over before the lines after them have said. The flywheel begins that field.)
static void the_levels_jump_with_a_flashing_light() {
  const Signal plain = video(1.0, on_the_grid, nullptr);
  const DecoderStats steady = decode(plain);
  CHECK(steady.acquisitions == 1 && steady.losses == 0);
  for (const double depths : {0.6, 1.3, 2.0, -0.6, -1.3, -2.0})
    for (const double tau : {0.0, 0.5e-3, 2e-3, 5e-3}) {
      const DecoderStats st = decode(moved(plain, flashing(depths * (kBlank - kTip), tau)));
      const long long missed = static_cast<long long>(steady.lines_locked) - static_cast<long long>(st.lines_locked);
      const bool ok = st.acquisitions == 1 && st.losses == 0 && st.rephases == 0 && st.fields == steady.fields &&
                      missed <= kJumpsASecond * 8 &&
                      (tau > 0 || st.level_jumps == kJumpsASecond);  // (a slow move is partly followed without)
      CHECK(ok);
      if (shown(ok))
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
    if (shown(ok))
      std::fprintf(stderr, "  %+.1f depths: %zu frames (steady: %zu), %d rows off their grey\n", depths, f.size(),
                          steady.size(), off);
  }
}

// ---------------------------------------------------------------------------------------- the vertical flywheel

// The rhythm by itself (field_rhythm.h), on line numbers alone: where a field begins for the starts found, or for
// none found. NTSC: an odd field (kind 0) of 262 lines, then an even one (kind 1) of 263.
static void the_rhythm_on_line_numbers() {
  using Start = std::optional<FieldStart>;
  const int64_t lines = 262;
  const FieldStart none{-1, 0}, odd{0, 100}, even{1, 362}, next_odd{0, 625};
  {
    FieldRhythm r;
    CHECK(r.overdue(none, 5000) == std::nullopt);  // no rhythm before the first start
    // the standard's rhythm is taken as read from the first start found
    CHECK(r.found(none, odd, lines) == Start(odd));
    CHECK(r.overdue(odd, even.line + 1) == std::nullopt);  // a start may yet be found a line late
    CHECK(r.overdue(odd, even.line + 2) == Start(even));   // none two lines on: the field begins where it was due
    CHECK(r.found(odd, even, lines) == Start(even));
    // a start a line or two off, or on its line as the other kind, is put where it was due
    CHECK(r.found(even, FieldStart{0, 626}, lines) == Start(next_odd));
    CHECK(r.found(even, FieldStart{0, 627}, lines) == Start(next_odd));
    CHECK(r.found(even, FieldStart{1, 625}, lines) == Start(next_odd));
    CHECK(r.found(even, FieldStart{0, 624}, lines) == Start(next_odd));
    CHECK(r.found(even, next_odd, lines) == Start(next_odd));
    // far off: passed over. The third in a row is where the field begins, and the standard's rhythm is forgotten
    CHECK(r.found(even, FieldStart{0, 500}, lines) == std::nullopt);
    CHECK(r.found(even, FieldStart{0, 501}, lines) == std::nullopt);
    CHECK(r.found(even, FieldStart{0, 502}, lines) == Start(FieldStart{0, 502}));
    CHECK(r.overdue(FieldStart{0, 502}, 5000) == std::nullopt);
    // the source's own is learnt once it is seen twice running: here fields that are all alike, 262 lines each,
    // the first of them found three fields on
    CHECK(r.found(FieldStart{0, 502}, FieldStart{0, 1288}, lines) == Start(FieldStart{0, 1288}));
    CHECK(r.overdue(FieldStart{0, 1288}, 5000) == std::nullopt);
    CHECK(r.found(FieldStart{0, 1288}, FieldStart{0, 1550}, lines) == Start(FieldStart{0, 1550}));
    CHECK(r.overdue(FieldStart{0, 1550}, 1812 + 2) == Start(FieldStart{0, 1812}));
    // ... and the standard's again when it leads from one start found to the next
    r.forget();
    CHECK(r.found(FieldStart{0, 1550}, FieldStart{1, 1812}, lines) == Start(FieldStart{1, 1812}));
    CHECK(r.overdue(FieldStart{1, 1812}, 2075 + 2) == Start(FieldStart{0, 2075}));
  }
  {
    // a jump in the stream loses the count of lines: nothing is due, and the next start found is taken as it is
    FieldRhythm r;
    CHECK(r.found(none, odd, lines) == Start(odd));
    r.forget();
    CHECK(r.overdue(odd, 5000) == std::nullopt);
    CHECK(r.found(odd, FieldStart{1, 437}, lines) == Start(FieldStart{1, 437}));
    CHECK(r.overdue(FieldStart{1, 437}, 700 + 2) == Start(FieldStart{0, 700}));
  }
  {
    // ... and in a faint signal, whose next vertical sync may be many fields off, the fields go on as they were
    // until one shows: that one is taken as it is, and the rhythm runs on from it
    FieldRhythm r;
    CHECK(r.found(none, odd, lines) == Start(odd));
    r.doubt();
    CHECK(r.overdue(odd, even.line + 2) == Start(even));
    CHECK(r.seen(even, FieldStart{0, 600}, lines, FieldRhythm::Signal::held) == Start(FieldStart{0, 600}));
    CHECK(r.overdue(FieldStart{0, 600}, 862 + 2) == Start(FieldStart{1, 862}));
    CHECK(r.seen(FieldStart{0, 600}, FieldStart{1, 880}, lines, FieldRhythm::Signal::held) == std::nullopt);  // (no longer)
  }
  {
    // Vertical syncs seen off the rhythm (FieldRhythm::seen), 15 lines early here and ten fields apart: in a faint
    // signal, which shows few of them, the second is where the fields begin; in any other it takes three, and
    // each within eight fields of the one before.
    for (const FieldRhythm::Signal signal : {FieldRhythm::Signal::held, FieldRhythm::Signal::faint, FieldRhythm::Signal::steady}) {
      const bool faint = signal != FieldRhythm::Signal::steady;
      FieldRhythm r;
      CHECK(r.found(none, odd, lines) == Start(odd));
      CHECK(r.seen(even, FieldStart{0, 610}, lines, signal) == std::nullopt);
      const Start second = r.seen(FieldStart{1, 2987}, FieldStart{0, 3235}, lines, signal);
      CHECK(second == (faint ? Start(FieldStart{0, 3235}) : std::nullopt));
    }
  }
}

// The signal with the vertical sync of some fields damaged: of its 6 half lines of broad pulses, the first
// gone(field) are blanked. 6: the field has no vertical sync; 2 or 3: it shows a line late; 1: it shows as the
// other kind of field's.
static Signal without_vertical_sync(Signal s, const std::function<int(int64_t)> &gone,
                                    int per_field = kHalfLinesPerField) {
  const int64_t fields = static_cast<int64_t>(static_cast<double>(s.d.size()) / (kT / 2)) / per_field + 1;
  for (int64_t f = 0; f < fields; f++) {
    const double t = static_cast<double>(f * per_field + 6) * (kT / 2);
    s.put(t, t + gone(f) * (kT / 2), kBlank);
  }
  return s;
}

// The decoder without its vertical flywheel, without its following of level jumps, with neither. Each without the
// hold as well, which keeps the lock through a jump of the levels too (a few lines later than following them does);
// kNoHold: without the hold alone.
static const DecoderOpts kNoFlywheel{.flywheel = false, .hold = false}, kNoLevels{.follow_levels = false, .hold = false},
    kNeither{.follow_levels = false, .flywheel = false, .hold = false}, kNoHold{.hold = false};

// back: only that many of the last pictures of each are compared (0: all of them, and as many of them). within: how
// far a pixel's red, green or blue may differ (the same signal at other sample numbers is drawn the same to a
// rounding; a line out of place in these pictures is off by most of the range).
static bool same_pictures(const std::vector<Picture> &a, const std::vector<Picture> &b, size_t back = 0, int within = 0) {
  if (back == 0 && a.size() != b.size()) return false;
  const size_t n = back ? back : a.size();
  if (a.size() < n || b.size() < n) return false;
  for (size_t i = 1; i <= n; i++) {
    const std::vector<uint8_t> &x = a[a.size() - i].px, &y = b[b.size() - i].px;
    if (x.size() != y.size()) return false;
    if (within == 0 && x != y) return false;
    for (size_t k = 0; within > 0 && k < x.size(); k++)
      if (std::abs(x[k] - y[k]) > within) return false;
  }
  return true;
}

// Two fields of three without their vertical sync: every picture comes out, and the same as with it. Interlaced
// fields, the rhythm taken as read; and fields that are all alike, a rhythm the source has to show first (its first
// few fields are begun where the standard's would be, then its own is learnt: after that its pictures are the ones
// a decoder without the flywheel draws). Without the flywheel the fields with no vertical sync are not drawn.
static void fields_coast_through_a_missing_vertical_sync() {
  for (const int per_field : {525, 524}) {
    const Signal s = moving_stripes(1.0, per_field);
    const int from = per_field == 525 ? 8 : 20;  // once the rhythm is had
    const auto two_of_three = [](int64_t f0) { return [f0](int64_t f) { return f >= f0 && f % 3 != 0 ? 6 : 0; }; };
    const Signal hit = without_vertical_sync(s, two_of_three(from), per_field);
    const std::vector<Picture> whole = frames_of(s, PictureOpts{});
    const std::vector<Picture> coasted = frames_of(hit, PictureOpts{}, nullptr, DecoderOpts{}, kLateCoasted);
    // ... and gone from the very start: the rhythm is had from the fields that are found, a few fields apart
    const DecoderStats sparse = decode(without_vertical_sync(s, two_of_three(0), per_field));
    const bool sparse_ok = sparse.frames + 30 >= whole.size() && sparse.losses == 0;
    CHECK(sparse_ok);
    if (shown(sparse_ok))
      std::fprintf(stderr, "  %d half lines a field, two vertical syncs in three gone from the start: %llu pictures of "
                   "%zu (%llu coasted)\n", per_field, (unsigned long long)sparse.frames, whole.size(),
                   (unsigned long long)sparse.fields_coasted);
    const DecoderStats st = decode(hit), bare = decode(hit, kNoFlywheel);
    const bool learnt = same_pictures(whole, frames_of(s, PictureOpts{}, nullptr, kNoFlywheel), 40);
    const bool ok = whole.size() >= 50 && same_pictures(whole, coasted) && learnt && st.fields_coasted >= 20 &&
                    st.acquisitions == 1 && st.losses == 0 && bare.frames + 20 <= whole.size() &&
                    bare.fields_coasted == 0;
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  %d half lines a field: %zu pictures (the last 40 as without the flywheel: %d), %zu with "
                   "two vertical syncs in three gone (%llu coasted), %llu without the flywheel\n", per_field,
                   whole.size(), learnt, coasted.size(), (unsigned long long)st.fields_coasted,
                   (unsigned long long)bare.frames);
  }
}

// The vertical sync damaged at its start, so that it is found a line late, or as the other kind of field's: the
// field begins where it is due all the same, as the kind it is, and the pictures are the same. Without the
// flywheel they are not: the field is drawn a line off, or into the other field's lines.
static void a_field_start_found_off_is_put_right() {
  const Signal s = moving_stripes(1.0);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  for (const int gone : {1, 2, 3}) {
    const Signal hit = without_vertical_sync(s, [gone](int64_t f) { return f >= 8 && f % 3 != 0 ? gone : 0; });
    const bool same = same_pictures(whole, frames_of(hit, PictureOpts{}, nullptr, DecoderOpts{}, kLateCoasted));
    const bool bare = same_pictures(whole, frames_of(hit, PictureOpts{}, nullptr, kNoFlywheel, kLateCoasted));
    const DecoderStats st = decode(hit);
    const bool ok = same && !bare && st.starts_put_right + st.fields_coasted >= 15 && st.losses == 0;
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  %d half lines of the vertical sync gone: pictures the same %d (without the flywheel %d), "
                   "%llu starts put right, %llu fields coasted\n", gone, same, bare,
                   (unsigned long long)st.starts_put_right, (unsigned long long)st.fields_coasted);
  }
}

// The source's fields move: 125 lines of it are cut out (a whole number of samples, so the line grid is unbroken).
// The field starts found are then all elsewhere than the rhythm says. Three of them in a row and they are
// believed: the pictures after that are the source's.
static void a_new_rhythm_is_believed() {
  const Signal s = moving_stripes(1.0);
  const size_t at = static_cast<size_t>(std::llround(40 * 125 * kT)), cut = static_cast<size_t>(std::llround(125 * kT));
  Signal moved_on;
  moved_on.d.assign(s.d.begin(), s.d.begin() + at);
  moved_on.d.insert(moved_on.d.end(), s.d.begin() + at + cut, s.d.end());
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  const std::vector<Picture> after = frames_of(moved_on, PictureOpts{}, nullptr, DecoderOpts{}, kLateCoasted);
  const DecoderStats st = decode(moved_on);
  const bool same = same_pictures(whole, after, 30, 3), ok = same && after.size() + 6 >= whole.size() && st.losses == 0;
  CHECK(ok);
  if (shown(ok))
    std::fprintf(stderr, "  125 lines cut out: %zu pictures (whole: %zu), the last 30 the same %d, %llu coasted\n",
                 after.size(), whole.size(), same, (unsigned long long)st.fields_coasted);
}

// The two together and each by itself, on the light flashing at 7 Hz: following the levels keeps the lock, and the
// flywheel begins the field whose vertical sync a jump landed on. The flywheel alone does not help there: with
// the lock gone at every flash there is no rhythm left to go by.
static void levels_and_flywheel_together_and_apart() {
  const Signal plain = video(1.0, on_the_grid, nullptr);
  const DecoderStats steady = decode(plain);
  for (const double depths : {1.3, -1.3, -2.0}) {
    const Signal s = moved(plain, flashing(depths * (kBlank - kTip), 0.5e-3));
    const DecoderStats both = decode(s), levels = decode(s, kNoFlywheel),
                       fly = decode(s, kNoLevels), neither = decode(s, kNeither);
    const bool ok = both.losses == 0 && both.fields == steady.fields && levels.losses == 0 &&
                    levels.fields + 1 >= steady.fields && fly.losses >= 5 && neither.losses >= 5 &&
                    fly.frames + 20 < both.frames;
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  %+.1f depths: fields / losses with both %llu / %llu, levels alone %llu / %llu, flywheel "
                   "alone %llu / %llu, neither %llu / %llu (steady: %llu)\n", depths,
                   (unsigned long long)both.fields, (unsigned long long)both.losses,
                   (unsigned long long)levels.fields, (unsigned long long)levels.losses,
                   (unsigned long long)fly.fields, (unsigned long long)fly.losses,
                   (unsigned long long)neither.fields, (unsigned long long)neither.losses,
                   (unsigned long long)steady.fields);
  }
}

// ---------------------------------------------------------------------------------------- a weak signal

// The fold by itself (fold.h), on a fold made here: blanking at 0, an hsync 4.7 us long down at -2 that begins at
// `edge` (bins, to a fraction: a bin an edge falls in has its share of both levels), picture from 9.4 us after it.
// The hsync is found by its shape, its edge to a hundredth of a bin, and its levels. In a whole line's fold too,
// where it lies across the fold's ends, and where something dark in the picture looks like a pulse as well (less
// deep than the sync is): the hsync is the line's best pulse and the lowest thing in it, and the dark bar, looked at
// by itself, is not. A fold of nothing has none.
static void the_fold_finds_the_hsync() {
  const auto level = [](Fold &f, double from, double to, float v) {  // bins [from, to) at v, wrapping
    const int K = static_cast<int>(f.v.size());
    for (int i = static_cast<int>(std::floor(from)); i < to; i++) {
      const double share = std::min<double>(i + 1, to) - std::max<double>(i, from);
      float &bin = f.v[static_cast<size_t>(((i % K) + K) % K)];
      bin += static_cast<float>(share) * (v - bin);
    }
  };
  {
    Fold f;  // a stretch around where the hsync is due, a sample to a bin
    f.shape(static_cast<size_t>(21 * kUs), kFs, 1, false);
    const double edge = 7 * kUs + 3.3;
    level(f, edge, edge + 4.7 * kUs, kTip);
    const std::optional<FoldPulse> p = f.pulse(f.bins(2.5), f.bins(11.5));
    CHECK(p && std::fabs(p->edge - edge) < 0.01 && std::fabs(p->tip - kTip) < 0.01f && std::fabs(p->blank - kBlank) < 0.01f &&
          std::fabs(p->depth - (kBlank - kTip)) < 0.01f);
    Fold none;
    none.shape(f.v.size(), kFs, 1, false);
    CHECK(!none.pulse(none.bins(2.5), none.bins(11.5)));
  }
  {
    Fold f;  // a whole line, four samples to a bin, the hsync two bins before its end
    const int K = static_cast<int>(kT / 4);
    f.shape(static_cast<size_t>(K), kFs, 4, true);
    const double edge = K - 2.4, bar = f.bins(30);
    level(f, edge, edge + f.bins(4.7), kTip);
    level(f, edge + f.bins(9.4), edge + f.bins(62), kGrey);
    level(f, bar, bar + f.bins(5), 0.2f);  // something dark in the picture, as long as a pulse (not as deep)
    const std::optional<FoldPulse> p = f.pulse(0, K - 1), dark = f.pulse(static_cast<int>(bar) - 4, static_cast<int>(bar) + 4);
    CHECK(p && std::fabs(p->edge - edge) < 0.01 && std::fabs(p->tip - kTip) < 0.01f && p->lowest_there());
    // (the bar passes for a pulse by its shape, but lies well above the lowest thing in the line)
    CHECK(dark && std::fabs(dark->edge - bar) < 0.5 && dark->tip > p->lowest + 0.25f * dark->depth);
  }
}

// Noisy or not goes by how much of the time (sync_noise.h): one noisy moment in ten is not noise; noisy most of
// the time is, after some thirty measurements at a fiftieth each; between the two limits it stays what it was;
// and it is clean again once it has been over the upper one for four fifths of the time.
static void noisy_by_how_much_of_the_time() {
  NoisyLatch l{12, 16};
  for (int i = 0; i < 300; i++) l.update(i % 10 == 0 ? 5.f : 30.f, 0.02f);
  CHECK(!l.noisy);
  int n = 0;
  while (!l.noisy && n < 1000) l.update(5.f, 0.02f), n++;
  CHECK(l.noisy && n > 20 && n < 40);
  for (int i = 0; i < 300; i++) l.update(14.f, 0.02f);
  CHECK(l.noisy);
  n = 0;
  while (l.noisy && n < 1000) l.update(30.f, 0.02f), n++;
  CHECK(!l.noisy && n > 60 && n < 100);
}

// The signal as a receiver far from the transmitter gives it, from t0 to t1 (seconds). The discriminator is
// captured by the noise part of the time, which pulls every level towards the middle of the band (left: how much
// of the signal's swing is left), and the noise is on top (sd MHz a sample, white: about a third of it gets through
// the sync path's low-pass).
static Signal faded(Signal s, double t0, double t1, float left, float sd, float centre = 0.5f) {
  Noise noise{0x9e3779b97f4a7c15ull};
  const size_t a = static_cast<size_t>(t0 * kFs), b = std::min(s.d.size(), static_cast<size_t>(t1 * kFs));
  for (size_t i = a; i < b; i += 2) {
    const auto [n0, n1] = noise.normal2(sd);
    s.d[i] = centre + left * (s.d[i] - centre) + static_cast<float>(n0);
    if (i + 1 < b) s.d[i + 1] = centre + left * (s.d[i + 1] - centre) + static_cast<float>(n1);
  }
  return s;
}

// How many of the pictures in got begin where one of the clean signal's does, to 0.3 of a line (a line off, or the
// other field's half line off, is not in place): their lines and their fields are where the signal's are.
static size_t in_place(const std::vector<Picture> &clean, const std::vector<Picture> &got) {
  size_t n = 0;
  for (const Picture &g : got) {
    double best = 1e9;
    for (const Picture &c : clean) best = std::min(best, std::fabs(c.times.t_first - g.times.t_first));
    if (best < 0.3 * kT / kFs) n++;
  }
  return n;
}

// 0.4 s of a signal in which a line's own hsync is a matter of chance (noise through the sync path as large as
// half the sync is deep, or as large as all of it, and the sync half as deep as it was): the lock is not lost,
// every picture comes out and begins where the signal's does, and afterwards the pictures are the clean signal's
// again, to the pixel: no line and no field was slipped. Without the hold the lock goes, and pictures with it.
static void the_grid_is_held_through_a_fade() {
  const Signal s = moving_stripes(1.0);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  for (const float sd : {3.f, 6.f}) {
    const Signal f = faded(s, 0.3, 0.7, 0.5f, sd);
    const std::vector<Picture> got = frames_of(f, PictureOpts{}, nullptr, DecoderOpts{}, 18);
    const DecoderStats st = decode(f), bare = decode(f, kNoHold);
    const size_t placed = in_place(whole, got);
    const bool same = same_pictures(whole, got, 8, 3);
    const bool ok = st.losses == 0 && st.acquisitions == 1 && st.rephases == 0 && st.holds >= 1 && !st.holding &&
                    got.size() == whole.size() && placed == got.size() && same && bare.losses >= 1 &&
                    bare.frames + 5 <= whole.size();
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  a fade, noise %.0f MHz: %zu pictures of %zu, %zu in place, the last 8 the same %d; %llu holds, "
                   "%llu losses, %llu re-phases, %llu coasted; without the hold %llu pictures, %llu losses\n", sd,
                   got.size(), whole.size(), placed, same, (unsigned long long)st.holds, (unsigned long long)st.losses,
                   (unsigned long long)st.rephases, (unsigned long long)st.fields_coasted,
                   (unsigned long long)bare.frames, (unsigned long long)bare.losses);
  }
}

// Such a signal from the first sample: it is acquired, the fold holds its lines, and its fields are found by what
// their half lines are (the broad pulses do not show as one run under the threshold in that noise): nearly every
// picture comes out, in its place. Without the hold next to none does.
static void a_weak_signal_from_the_start() {
  const Signal s = moving_stripes(1.0);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  const Signal f = faded(s, 0, 1.0, 0.5f, 6.f);
  const std::vector<Picture> got = frames_of(f, PictureOpts{}, nullptr, DecoderOpts{}, 18);
  const DecoderStats st = decode(f), bare = decode(f, kNoHold);
  const size_t placed = in_place(whole, got);
  const bool ok = st.losses == 0 && st.holding && got.size() + 8 >= whole.size() && placed + 2 >= got.size() &&
                  bare.frames * 2 < got.size();
  CHECK(ok);
  if (shown(ok))
    std::fprintf(stderr, "  weak from the start: %zu pictures of %zu, %zu in place; %llu acquisitions, %llu losses, %llu "
                 "holds; without the hold %llu pictures\n", got.size(), whole.size(), placed,
                 (unsigned long long)st.acquisitions, (unsigned long long)st.losses, (unsigned long long)st.holds,
                 (unsigned long long)bare.frames);
}

// No signal at all, only noise, for 0.2 s: the grid runs on at the line period, pictures of noise come out in
// their places, and when the signal is back so are its pictures, to the pixel. For 0.8 s: the lock is given up
// after half a second, and found again when the signal is back.
static void nothing_for_a_moment_and_for_long() {
  const Signal s = moving_stripes(1.8);
  const std::vector<Picture> whole = frames_of(s, PictureOpts{});
  const Signal moment = faded(s, 0.6, 0.8, 0.f, 6.f), lng = faded(s, 0.6, 1.4, 0.f, 6.f);
  const std::vector<Picture> got = frames_of(moment, PictureOpts{}, nullptr, DecoderOpts{}, 18);
  const DecoderStats st = decode(moment), gone = decode(lng);
  const size_t placed = in_place(whole, got);
  const bool same = same_pictures(whole, got, 8, 3);
  const bool ok = st.losses == 0 && st.acquisitions == 1 && st.rephases == 0 && got.size() == whole.size() &&
                  placed == got.size() && same;
  CHECK(ok);
  CHECK(gone.losses == 1 && gone.acquisitions == 2 && !gone.acquiring && gone.lock_recent > 0.9f);
  if (shown(ok && gone.losses == 1))
    std::fprintf(stderr, "  nothing for 0.2 s: %zu pictures of %zu, %zu in place, the last 8 the same %d, %llu losses, %llu "
                 "re-phases; for 0.8 s: %llu losses, %llu acquisitions, lock %.2f\n", got.size(), whole.size(), placed,
                 same, (unsigned long long)st.losses, (unsigned long long)st.rephases, (unsigned long long)gone.losses,
                 (unsigned long long)gone.acquisitions, gone.lock_recent);
}

// A block of samples missing from such a signal, as when the phone drops one: 30 lines and a part of a line. From
// there on the signal's lines are not where the grid has them, and its fields begin 30 lines early. The fold of
// the whole line shows where the hsync has gone, and the grid is put there; the pictures go on coming meanwhile,
// at the rhythm they had, and 0.2 s on they begin where the signal's do. Two ways for it to go: nothing like a
// pulse is left where the grid is (stripes, the signal 20 us early); or there is the dark edge of something in the
// picture there, which passes for a pulse (a bar 15 us into each line, the signal 15 us early), and the hsync,
// lower than anything in the picture, is the whole line's best pulse all the same.
static void a_block_missing_from_a_weak_signal() {
  struct Case {
    const char *name;
    Signal s;
    double us;
  };
  const Signal bar = video(1.6, on_the_grid, [](Signal &s, int64_t, double start) {
    s.put(start + 15 * kUs, start + 20 * kUs, 0.2f);
  });
  for (Case c : {Case{"stripes, 20 us early", moving_stripes(1.6), 20}, Case{"a dark bar where the grid is", bar, 15}}) {
    const size_t at = static_cast<size_t>(0.6 * kFs);
    c.s.d.erase(c.s.d.begin() + at, c.s.d.begin() + at + static_cast<size_t>(30 * kT + c.us * kUs));
    // (as a strong signal, its lines found again at once, no field is begun between the jump and the next
    // vertical sync found: the picture in hand at the jump comes out a field late)
    const std::vector<Picture> whole = frames_of(c.s, PictureOpts{}, nullptr, DecoderOpts{}, 300);
    const Signal f = faded(c.s, 0, 1.6, 0.5f, 6.f);
    const std::vector<Picture> got = frames_of(f, PictureOpts{}, nullptr, DecoderOpts{}, 18);
    const DecoderStats st = decode(f);
    // (when they began, without their pixels)
    const auto from = [](const std::vector<Picture> &v, double t0, double t1) {
      std::vector<Picture> out;
      for (const Picture &p : v)
        if (p.times.t_first >= kT0 + t0 && p.times.t_first < kT0 + t1) out.push_back(Picture{{}, p.times});
      return out;
    };
    const std::vector<Picture> after = from(got, 0.8, 1.6);
    const size_t meanwhile = from(got, 0.6, 0.8).size(), due = from(whole, 0.8, 1.6).size(), placed = in_place(whole, after);
    const bool ok = st.losses == 0 && st.acquisitions == 1 && st.rephases >= 1 && st.rephases <= 2 && meanwhile >= 10 &&
                    after.size() + 1 >= due && placed + 1 >= after.size();
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  a block missing, %s: %llu re-phases, %llu losses; %zu pictures in the 0.2 s after it, then %zu "
                   "of %zu, %zu in place\n", c.name, (unsigned long long)st.rephases, (unsigned long long)st.losses,
                   meanwhile, after.size(), due, placed);
  }
}

// ---------------------------------------------------------------------------------------- colour in noise

// The grey picture in one colour: a burst on every picture line's back porch (40 IRE peak to peak, as the sync is
// deep) and a subcarrier across the picture, `hue` radians from the burst. NTSC: 227.5 cycles a line, here on the
// samples' clock.
static Signal coloured(double seconds, double hue) {
  return video(seconds, on_the_grid, [hue](Signal &s, int64_t, double start) {
    const double w = 2 * M_PI * 227.5 / kT;
    const auto wave = [&](double from_us, double to_us, double amp, double ph) {
      const int64_t a = std::max<int64_t>(0, std::llround(start + from_us * kUs));
      const int64_t b = std::min<int64_t>(static_cast<int64_t>(s.d.size()), std::llround(start + to_us * kUs));
      for (int64_t i = a; i < b; i++) s.d[i] += static_cast<float>(amp * std::cos(w * static_cast<double>(i) + ph));
    };
    wave(5.3, 7.8, 1.0, M_PI);
    wave(9.4, 62.0, 0.6, M_PI + hue);
  });
}

// What colour the middle of the last `back` pictures has on average: red and blue against green, 0..255.
static std::pair<double, double> tint(const std::vector<Picture> &v, size_t back) {
  double rg = 0, bg = 0, n = 0;
  for (size_t k = 1; k <= back && k <= v.size(); k++) {
    const std::vector<uint8_t> &px = v[v.size() - k].px;
    const size_t w = 720, h = px.size() / 4 / w;
    for (size_t y = h / 4; y < 3 * h / 4; y += 2)
      for (size_t x = w / 4; x < 3 * w / 4; x += 2) {
        const uint8_t *p = &px[4 * (y * w + x)];
        rg += p[0] - p[1];
        bg += p[2] - p[1];
        n++;
      }
  }
  return {rg / std::max(n, 1.0), bg / std::max(n, 1.0)};
}

// A noisy signal keeps its colour. One line's burst is mostly noise then, and the decoder used to show no colour
// (with the hold), or bands of wrong colour (without): now the bursts are averaged over many lines, and the
// picture's colour is the clean signal's: the same hue, and the paler the larger the noise (a tenth of the clean
// signal's saturation where the noise through the colour path is some three times the burst). A noisy signal
// without a burst gets no colour from its noise.
static void a_noisy_signal_keeps_its_colour() {
  for (const double hue : {0.6, 2.4, 4.4}) {
    const Signal s = coloured(1.0, hue);
    const std::vector<Picture> whole = frames_of(s, PictureOpts{});
    const auto [r0, b0] = tint(whole, 8);
    const double size0 = std::hypot(r0, b0);
    double left[2] = {};
    bool ok = clean().colour == false && size0 > 20;
    int i = 0;
    for (const float sd : {1.5f, 3.f}) {
      const Signal f = faded(s, 0, 1.0, 0.5f, sd);
      const std::vector<Picture> got = frames_of(f, PictureOpts{}, nullptr, DecoderOpts{}, 18);
      const DecoderStats st = decode(f);
      const auto [r1, b1] = tint(got, 16);
      const double turn = std::fabs(std::remainder(std::atan2(b1, r1) - std::atan2(b0, r0), 2 * M_PI));
      left[i] = std::hypot(r1, b1) / std::max(size0, 1e-9);
      const bool one = st.colour && got.size() + 2 >= whole.size() && turn < 0.3;
      ok = ok && one;
      if (shown(one))
        std::fprintf(stderr, "  colour in noise %.1f MHz, hue %.1f: clean red-green %+.0f blue-green %+.0f; noisy %+.0f %+.0f "
                     "(%.0f %% of the size, %.2f rad off), colour %s, %zu pictures of %zu\n", sd, hue, r0, b0, r1, b1,
                     100 * left[i], turn, st.colour ? "on" : "off", got.size(), whole.size());
      i++;
    }
    CHECK(ok);
    CHECK(left[0] < 1.1 && left[0] > left[1] && left[1] > 0.05);
  }
  const DecoderStats grey = decode(faded(video(1.0, on_the_grid, nullptr), 0, 1.0, 0.5f, 3.f));
  CHECK(!grey.colour && grey.holding);
}

// ---------------------------------------------------------------------------------------- the channel filter

// The signal on its carrier, as the adapter gives it: FM with the discriminator's output as its frequency, the
// carrier `above` MHz over the stream's middle, and white noise `cnr_db` under the carrier (in the whole stream).
// (the signal's levels here are the test signal's: sync tip at -2 MHz, white near +4)
static std::vector<cf> on_a_carrier(const Signal &s, double cnr_db, double above) {
  std::vector<cf> x(s.d.size());
  Noise noise{0x2545f4914f6cdd1dull};
  const double sd = cnr_db > 90 ? 0 : std::sqrt(0.5 / std::pow(10, cnr_db / 10));
  double ph = 0;
  for (size_t i = 0; i < x.size(); i++) {
    ph = std::remainder(ph + 2 * M_PI * (s.d[i] + above) * 1e6 / kFs, 2 * M_PI);
    const auto [nr, ni] = noise.normal2(sd);
    x[i] = cf(static_cast<float>(std::cos(ph) + nr), static_cast<float>(std::sin(ph) + ni));
  }
  return x;
}

struct Watched {
  DecoderStats st;
  std::vector<Picture> pics;
  float widest = 0;  // the channel filter at its widest
};
// The stream through a whole front end (the discriminator too), the carrier `above` MHz over its middle
static Watched watch(const std::vector<cf> &x, double above, const DecoderOpts &o = DecoderOpts{}) {
  Watched w;
  FrontEnd fe(kFs, above * 1e6, [&](const Frame &f) {
    if (!f.finished()) return;
    const size_t bytes = static_cast<size_t>(f.width) * f.height * 4;
    w.pics.push_back(Picture{std::vector<uint8_t>(f.data, f.data + bytes), f.times});
  }, o);
  std::vector<float> d;
  std::vector<uint8_t> bad;
  for (size_t at = 0; at < x.size(); at += 4096) {
    const size_t n = std::min<size_t>(4096, x.size() - at);
    fe.demod(x.data() + at, n, d, bad);
    fe.decode(d, bad, kT0 + static_cast<double>(at + n - 1) / kFs);
    w.widest = std::max(w.widest, fe.stats().filter_mhz);
  }
  w.st = fe.stats();
  return w;
}

// A clean signal never has the filter in, and its pictures are the ones a front end without the filter draws (the
// stream only comes 11 samples later). A noisy one has it in within a tenth of a second, as wide as six of the
// transmitter's sync depths, not retuned over and over, with the decoder's levels where they were, and more of
// the lines find their hsync than without it. From the first sample too, when the signal has never been better:
// the filter is put by a weak signal's own shrunken levels, and does not run off with them.
static void a_noisy_signal_gets_a_channel_filter() {
  DecoderOpts none;
  none.channel_filter = false;
  const Signal s = moving_stripes(0.8);
  const double above = 1.5;
  {
    const std::vector<cf> x = on_a_carrier(s, 99, above);
    const Watched with = watch(x, above), without = watch(x, above, none);
    const bool ok = with.widest == 0 && with.st.filter_changes == 0 && with.st.losses == 0 &&
                    with.pics.size() == without.pics.size() && same_pictures(with.pics, without.pics, 20, 3);
    CHECK(ok);
    if (shown(ok))
      std::fprintf(stderr, "  clean: filter %.0f MHz at its widest, %llu changes, %zu pictures (without: %zu), sync snr "
                   "%.0f\n", with.widest, (unsigned long long)with.st.filter_changes, with.pics.size(),
                   without.pics.size(), with.st.sync_snr);
  }
  for (const double cnr : {6.0, 3.0}) {
    // clean for 0.25 s, then the noise; and the noise from the start
    for (const bool from_start : {false, true}) {
      std::vector<cf> x = on_a_carrier(s, cnr, above);
      if (!from_start) {
        const std::vector<cf> clean = on_a_carrier(s, 99, above);
        std::copy(clean.begin(), clean.begin() + static_cast<long>(0.25 * kFs), x.begin());
      }
      const Watched with = watch(x, above), without = watch(x, above, none);
      const float depth = kBlank - kTip;
      const bool ok = with.st.filter_mhz >= 7 && with.st.filter_mhz <= 13 && with.st.filter_changes <= 4 &&
                      with.st.losses == 0 && with.st.lines_locked > without.st.lines_locked &&
                      std::fabs(with.st.tip - without.st.tip) < 0.25f * depth &&
                      std::fabs(with.st.blank - without.st.blank) < 0.25f * depth &&
                      with.pics.size() + 3 >= without.pics.size();
      CHECK(ok);
      if (shown(ok))
        std::fprintf(stderr, "  carrier %.0f dB over the noise%s: filter %.0f MHz, %llu changes, lines locked %llu "
                     "(without: %llu), tip %.2f blank %.2f (without: %.2f %.2f), sync snr %.1f (without: %.1f), %zu "
                     "pictures (without: %zu), %llu losses\n", cnr, from_start ? " from the start" : "",
                     with.st.filter_mhz, (unsigned long long)with.st.filter_changes,
                     (unsigned long long)with.st.lines_locked, (unsigned long long)without.st.lines_locked,
                     with.st.tip, with.st.blank, without.st.tip, without.st.blank, with.st.sync_snr,
                     without.st.sync_snr, with.pics.size(), without.pics.size(), (unsigned long long)with.st.losses);
    }
  }
}

int main() {
  if (std::getenv("KP_TEST_DEBUG")) set_log_debug(true);
  locks_on_a_clean_signal();
  a_dip_ahead_of_the_hsync();
  a_gap_in_the_hsync();
  a_step_in_the_line_timing();
  strips_add_up_to_the_frames();
  frames_say_when_their_lines_were_taken();
  the_levels_jump_with_a_flashing_light();
  the_picture_keeps_its_grey_through_a_jump();
  the_rhythm_on_line_numbers();
  fields_coast_through_a_missing_vertical_sync();
  a_field_start_found_off_is_put_right();
  a_new_rhythm_is_believed();
  levels_and_flywheel_together_and_apart();
  the_fold_finds_the_hsync();
  noisy_by_how_much_of_the_time();
  the_grid_is_held_through_a_fade();
  a_weak_signal_from_the_start();
  nothing_for_a_moment_and_for_long();
  a_block_missing_from_a_weak_signal();
  a_noisy_signal_keeps_its_colour();
  a_noisy_signal_gets_a_channel_filter();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
