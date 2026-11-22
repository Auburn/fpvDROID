// kp_spectrum_tests: the Spectrum tab's band picture (BandMap, find_carriers) from sweep steps made here (ctest:
// "spectrum"). Plain checks, as in radio_tests.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "spectrum.h"

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

// ---------------------------------------------------------------------------------------- the air
struct Signal {
  double mhz, width_mhz;
  float db;   // over the noise
  bool flat;  // digital (a flat top, steep sides); else an FM video hump
};

// What the radio's sweep does (radio.cpp: sweep_plan, handle_sweep): retunes 30 MHz apart over lo..hi, every
// other pass moved by half a step, each step a 40 MS/s spectrum of 1024 bins (fftshifted, dB) with a gain of its
// own (the chip's AGC), the RX filter rolling off past 17 MHz, and noise. Its +-16.5 MHz go into the map.
static BandMap swept(const std::vector<Signal> &air, int passes, unsigned seed = 3) {
  constexpr double kFs = 40e6, kStep = 30e6, kUse = 16.5e6, kLo = 5635e6, kHi = 5955e6;
  constexpr int kN = 1024;
  std::mt19937 rng(seed);
  std::normal_distribution<float> noise(0, 1);
  std::uniform_real_distribution<float> gain(-6, 6);
  BandMap map(5300e6, 6000e6, 0.5e6);
  int64_t t = 1000;
  for (int pass = 0; pass < passes; pass++) {
    const double first = std::round((kLo + (pass % 2 ? 0 : kStep / 2)) / 5e6) * 5e6;
    for (double c = first; c - kStep / 2 < kHi; c += kStep) {
      std::vector<float> db(kN);
      const float g = gain(rng);
      for (int i = 0; i < kN; i++) {
        const double off = (i - kN / 2) * kFs / kN, f = (c + off) / 1e6;
        double p = 1.0;  // the noise, linear
        for (const auto &s : air) {
          const double x = (f - s.mhz) / (s.width_mhz / 2), lvl = std::pow(10.0, s.db / 10);
          p += s.flat ? (std::fabs(x) < 1 ? lvl : 0) : lvl * std::exp(-x * x * 1.6);
        }
        double v = 10 * std::log10(p) + g + 0.3 * noise(rng);
        if (std::fabs(off) > 17e6) v -= (std::fabs(off) - 17e6) / 1e6 * 6;
        db[i] = static_cast<float>(v);
      }
      map.add(db, kFs, c, kUse, t);
      t += 180;  // ~2 s a pass
    }
  }
  return map;
}

static std::vector<Carrier> carriers_of(const BandMap &map) {
  double first = 0;
  const auto db = map.get(5635e6, 5955e6, &first);
  return find_carriers(db, first, map.bin_hz());
}

// the carrier nearest mhz, or nullptr
static const Carrier *at(const std::vector<Carrier> &cs, double mhz) {
  const Carrier *best = nullptr;
  for (const auto &c : cs)
    if (std::fabs(c.mhz - mhz) < 3 && (!best || std::fabs(c.mhz - mhz) < std::fabs(best->mhz - mhz))) best = &c;
  return best;
}

// ---------------------------------------------------------------------------------------- the tests
// Nothing on the air: the picture is the floor, everywhere (every step's gain taken out), and nothing is found.
static void the_floor_is_flat_and_empty() {
  const BandMap map = swept({}, 4);
  double first = 0;
  const auto db = map.get(5635e6, 5955e6, &first);
  int unseen = 0;
  float lo = 1e9f, hi = -1e9f;
  for (float v : db) {
    if (v == BandMap::kNone) { unseen++; continue; }
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  CHECK(unseen == 0);   // the DC points of one pass are seen by the next
  CHECK(lo > -3 && hi < 3);
  CHECK(carriers_of(map).empty());
}

// An analog transmitter and DJI's 20 MHz and 40 MHz channels: found where they are, as wide as they are.
static void signals_are_found_where_and_as_wide_as_they_are() {
  const auto cs = carriers_of(swept({{5658.4, 12, 22, false}, {5735, 20, 25, true}, {5770, 40, 20, true}}, 4));
  CHECK(cs.size() == 3);
  const Carrier *analog = at(cs, 5658.4), *narrow = at(cs, 5735), *wide = at(cs, 5770);
  CHECK(analog && narrow && wide);
  if (narrow) CHECK(std::fabs(narrow->width_mhz - 20) < 2 && std::fabs(narrow->snr_db - 25) < 2);
  if (wide) CHECK(std::fabs(wide->width_mhz - 40) < 2 && std::fabs(wide->snr_db - 20) < 2);
  if (analog) CHECK(analog->width_mhz < 25);   // under the width that reads as one of DJI's wide modes
}

// A signal wider than a step's span (DJI O4 at 60 MHz, a weak 40 MHz one) leaves no noise in the steps inside it:
// their floor comes from the steps around them, and it is found whole, not as two edges.
static void a_signal_wider_than_a_step_is_found_whole() {
  auto cs = carriers_of(swept({{5794.5, 60, 18, true}}, 4));
  CHECK(cs.size() == 1);
  if (!cs.empty()) CHECK(std::fabs(cs[0].mhz - 5794.5) < 1.5 && std::fabs(cs[0].width_mhz - 60) < 2);
  cs = carriers_of(swept({{5878, 40, 12, true}}, 4));
  CHECK(cs.size() == 1);
  if (!cs.empty()) CHECK(std::fabs(cs[0].mhz - 5878) < 1.5 && std::fabs(cs[0].width_mhz - 40) < 2);
}

// Bins not seen again within the time asked are forgotten; a gap of up to 3 MHz between seen bins is bridged.
static void forget_and_bridge() {
  BandMap map(5600e6, 5700e6, 0.5e6);
  std::vector<float> db(1024, 0.f);
  map.add(db, 40e6, 5650e6, 16.5e6, 1000);
  double first = 0;
  auto out = map.get(5640e6, 5660e6, &first);
  bool all = true;
  for (float v : out) all = all && v != BandMap::kNone;
  CHECK(all);  // the DC point (+-0.75 MHz) is left out of the step, and bridged
  map.forget(5600e6, 5700e6, 2000);
  out = map.get(5640e6, 5660e6, &first);
  bool none = true;
  for (float v : out) none = none && v == BandMap::kNone;
  CHECK(none);
}

int main() {
  the_floor_is_flat_and_empty();
  signals_are_found_where_and_as_wide_as_they_are();
  a_signal_wider_than_a_step_is_found_whole();
  forget_and_bridge();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
