// kp_decoder_tests: the decoder's sync tracking on a signal made here (ctest: "decoder"). Plain checks, as in
// radio_tests.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
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

int main() {
  locks_on_a_clean_signal();
  a_dip_ahead_of_the_hsync();
  a_gap_in_the_hsync();
  a_step_in_the_line_timing();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
