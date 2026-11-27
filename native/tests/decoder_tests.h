// What the decoder's two test programs share (kp_decoder_tests: decoder_tests.cpp, kp_weak_signal_tests:
// weak_signal_tests.cpp): plain checks, as in radio_tests.cpp, a test signal made here, and what a decoder makes of it.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <utility>
#include <vector>

#include "video.h"

using namespace kp;

inline int g_checks = 0, g_failed = 0;
#define CHECK(c)                                                                  \
  do {                                                                            \
    g_checks++;                                                                   \
    if (!(c)) {                                                                   \
      g_failed++;                                                                 \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c);  \
    }                                                                             \
  } while (0)

// A case's numbers are printed when it failed, or for all cases with KP_TEST_VERBOSE set.
inline bool shown(bool ok) { return !ok || std::getenv("KP_TEST_VERBOSE"); }

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
constexpr double kFs = 20e6, kUs = kFs * 1e-6, kT = 63.5556 * kUs;  // samples
constexpr float kTip = -2.f, kBlank = 0.f, kGrey = 1.5f;
constexpr int kHalfLinesPerField = 525;

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
inline Signal video(double seconds, const std::function<double(int64_t)> &late,
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

inline double on_the_grid(int64_t) { return 0; }

inline DecoderStats decode(const Signal &s, const DecoderOpts &o = DecoderOpts{}) {
  FrontEnd fe(kFs, 0, [](const Frame &) {}, o);
  const std::vector<uint8_t> no_marks;
  std::vector<float> block;
  for (size_t at = 0; at < s.d.size(); at += 4096) {
    block.assign(s.d.begin() + at, s.d.begin() + std::min(s.d.size(), at + 4096));
    fe.decode(block, no_marks);
  }
  return fe.stats();
}

// What the decoder makes of 0.4 s of the clean signal.
inline const DecoderStats &clean() {
  static const DecoderStats st = decode(video(0.4, on_the_grid, nullptr));
  return st;
}

// A picture that differs from line to line, across each line, and from one field to the next (diagonal stripes
// moving down), so that a line out of place or a frame's line from the wrong field shows.
inline Signal moving_stripes(double seconds, int per_field = kHalfLinesPerField) {
  return video(seconds, on_the_grid, [](Signal &s, int64_t line, double start) {
    for (double us = 9.4; us < 62.0; us += 0.05) {
      const double ph = us * 0.9 + static_cast<double>(line) * 0.71;
      s.put(start + us * kUs, start + (us + 0.05) * kUs, static_cast<float>(0.3 + 4.0 * (0.5 + 0.5 * std::sin(ph))));
    }
  }, per_field);
}

// ---------------------------------------------------------------------------------------- its pictures

struct Picture {
  std::vector<uint8_t> px;
  LineTimes times;  // of the whole picture: its first line drawn, and its last
};

// What a decoder made of a signal: its finished pictures, and its counts at the end.
struct Watched {
  DecoderStats st;
  std::vector<Picture> pics;
  float widest = 0;  // the channel filter at its widest (a stream through the whole front end, the discriminator too)
};

constexpr double kT0 = 1000.0;  // when sample 0 was taken, as the decoder is told (sample i at kT0 + i / kFs)

// The signal through a decoder with the switches p, in blocks of 4096, each with the time its last sample was
// taken: its finished pictures and its counts. Checks as it goes that a picture's frames (several with strips) run
// down the picture without gaps or overlaps, and come out a strip's worth at a time as the lines are decoded.
// (late: how many lines after its newest line a frame may come out. kLateCoasted: where a field's vertical sync
// does not show, the field is waited on for two lines more, and a start that is found waits a line itself.)
constexpr double kLateCoasted = 13;
inline Watched watched(const Signal &s, PictureOpts p, int *strips_per_frame = nullptr,
                       const DecoderOpts &o = DecoderOpts{}, double late = 10) {
  Watched w;
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
    w.pics.push_back(Picture{std::vector<uint8_t>(f.data, f.data + bytes), LineTimes{t_top, f.times.t_last}});
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
  w.st = fe.stats();
  CHECK(order_ok);
  CHECK(early_ok);
  if (strips_per_frame) *strips_per_frame = most;
  return w;
}

// ... its pictures alone
inline std::vector<Picture> frames_of(const Signal &s, PictureOpts p, int *strips_per_frame = nullptr,
                                      const DecoderOpts &o = DecoderOpts{}, double late = 10) {
  return watched(s, p, strips_per_frame, o, late).pics;
}

// The decoder without its vertical flywheel, without its following of level jumps, with neither. Each without the
// hold as well, which keeps the lock through a jump of the levels too (a few lines later than following them does);
// kNoHold: without the hold alone.
constexpr DecoderOpts kNoFlywheel{.flywheel = false, .hold = false}, kNoLevels{.follow_levels = false, .hold = false},
    kNeither{.follow_levels = false, .flywheel = false, .hold = false}, kNoHold{.hold = false},
    kNoCoast{.coast = false};

// back: only that many of the last pictures of each are compared (0: all of them, and as many of them). within: how
// far a pixel's red, green or blue may differ (the same signal at other sample numbers is drawn the same to a
// rounding; a line out of place in these pictures is off by most of the range).
inline bool same_pictures(const std::vector<Picture> &a, const std::vector<Picture> &b, size_t back = 0, int within = 0) {
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

// The signal as a receiver far from the transmitter gives it, from t0 to t1 (seconds). The discriminator is
// captured by the noise part of the time, which pulls every level towards the middle of the band (left: how much
// of the signal's swing is left), and the noise is on top (sd MHz a sample, white: about a third of it gets through
// the sync path's low-pass).
inline Signal faded(Signal s, double t0, double t1, float left, float sd, float centre = 0.5f) {
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
inline size_t in_place(const std::vector<Picture> &clean, const std::vector<Picture> &got) {
  size_t n = 0;
  for (const Picture &g : got) {
    double best = 1e9;
    for (const Picture &c : clean) best = std::min(best, std::fabs(c.times.t_first - g.times.t_first));
    if (best < 0.3 * kT / kFs) n++;
  }
  return n;
}
