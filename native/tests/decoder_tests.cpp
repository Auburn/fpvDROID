// kp_decoder_tests: the decoder's sync tracking on a signal made here (ctest: "decoder"). Plain checks, as in
// radio_tests.cpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
  LineTimes times;
};

// The signal through a decoder with the switches p, in blocks of 4096: its frames (their pixels), and with strips its
// strips put together. Checks as it goes that each field's strips run down the picture without gaps or overlaps,
// come out a strip's worth at a time as the lines are decoded, and add up to the field's frame.
static std::vector<Picture> frames_of(const Signal &s, PictureOpts p, int *strips_per_frame = nullptr) {
  std::vector<Picture> out;
  std::vector<uint8_t> built;  // the strips so far, into a frame
  int next_y = 0, strips = 0, most = 0;
  uint64_t fed = 0;  // samples the decoder had been given before the block it is decoding
  bool order_ok = true, early_ok = true;
  FrontEnd fe(kFs, 0, [&](const Frame &f) {
    const size_t bytes = static_cast<size_t>(f.width) * f.height * 4;
    out.push_back(Picture{std::vector<uint8_t>(f.data, f.data + bytes), f.times});
    if (!p.strips) return;
    if (!f.in_strips) return;  // a field begun before the switch was on, or the decoder's first
    order_ok &= next_y == 0 && built.size() == bytes && std::memcmp(built.data(), f.data, bytes) == 0;
    most = std::max(most, strips);
    strips = 0;
  }, DecoderOpts{}, p);
  fe.set_strip_sink([&](const Strip &st) {
    const size_t rowb = static_cast<size_t>(st.width) * 4;
    built.resize(rowb * st.height);
    order_ok &= st.y0 == next_y && st.rows > 0 && st.y0 + st.rows <= st.height && st.last == (st.y0 + st.rows == st.height);
    std::memcpy(built.data() + st.y0 * rowb, st.data, st.rows * rowb);
    next_y = st.last ? 0 : st.y0 + st.rows;
    // drawn in order; out within a few lines of the newest of them being drawn (the block being decoded began
    // at fed: sparkle repair holds the last 3 lines back, the field's last strip waits for the field to end)
    early_ok &= st.times.first_at > 0 && st.times.first_at <= st.times.last_at && st.times.last_at + 10 * kT > fed;
    strips++;
  });
  const std::vector<uint8_t> no_marks;
  std::vector<float> block;
  for (size_t at = 0; at < s.d.size(); at += 4096) {
    block.assign(s.d.begin() + at, s.d.begin() + std::min(s.d.size(), at + 4096));
    fe.decode(block, no_marks);
    fed += block.size();
  }
  CHECK(order_ok);
  CHECK(early_ok);
  if (strips_per_frame) *strips_per_frame = most;
  return out;
}

// With strips on, the frames are the same as without them, and every field goes out in strips that add up to its
// frame: whatever smooth lines, sparkle repair and the 3-line comb (which draws each line a line late) do.
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
    bool same = plain.size() == in_strips.size();
    for (size_t i = 0; same && i < plain.size(); i++) same = plain[i].px == in_strips[i].px;
    CHECK(same);
    // NTSC: 240 field lines, 32 to a strip (a strip may wait 3 lines for sparkle repair, one for smooth lines)
    CHECK(per_frame >= 7 && per_frame <= 9);
    if (!same || per_frame < 7 || per_frame > 9)
      std::fprintf(stderr, "  smooth %d repair %d comb %d: %zu / %zu frames, %d strips a frame\n", p.smooth_lines,
                   p.repair, p.chroma_comb, plain.size(), in_strips.size(), per_frame);
  }
}

// A frame's times: its first line was drawn about a field (262.5 lines) before its last.
static void frames_say_when_their_lines_were_drawn() {
  const std::vector<Picture> f = frames_of(moving_stripes(0.3), PictureOpts{});
  CHECK(f.size() >= 8);
  for (size_t i = 2; i < f.size(); i++) {
    const double lines = static_cast<double>(f[i].times.last_at - f[i].times.first_at) / kT;
    CHECK(lines > 235 && lines < 245);  // first row to last row: 239 lines apart
    if (i > 2) CHECK(f[i].times.first_at > f[i - 1].times.last_at);
  }
}

int main() {
  locks_on_a_clean_signal();
  a_dip_ahead_of_the_hsync();
  a_gap_in_the_hsync();
  a_step_in_the_line_timing();
  strips_add_up_to_the_frames();
  frames_say_when_their_lines_were_drawn();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
