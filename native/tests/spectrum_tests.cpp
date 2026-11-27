// kp_spectrum_tests: the Spectrum tab's sweep plan (Sweep) and band picture (BandMap, find_carriers) from sweep steps
// made here, and what a scan step finds whole or cut (find_step_signals) (ctest: "spectrum"). Plain checks, as in
// radio_tests.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "scan.h"
#include "spectrum.h"
#include "sweep.h"

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

static constexpr double kFs = 40e6, kUse = ScanPlan::kEdge;  // a step's sample rate and the span of it used

// One retune's spectrum as the radio takes it: 40 MS/s, 1024 bins (fftshifted, dB), centred on the hardware centre
// centre_hz, with a gain of its own (the chip's AGC), the RX filter rolling off past 17 MHz, and noise.
static std::vector<float> step_db(const std::vector<Signal> &air, double centre_hz, float gain, std::mt19937 &rng) {
  constexpr int kN = 1024;
  std::normal_distribution<float> noise(0, 1);
  std::vector<float> db(kN);
  for (int i = 0; i < kN; i++) {
    const double off = (i - kN / 2) * kFs / kN, f = (centre_hz + off) / 1e6;
    double p = 1.0;  // the noise, linear
    for (const auto &s : air) {
      const double x = (f - s.mhz) / (s.width_mhz / 2), lvl = std::pow(10.0, s.db / 10);
      p += s.flat ? (std::fabs(x) < 1 ? lvl : 0) : lvl * std::exp(-x * x * 1.6);
    }
    double v = 10 * std::log10(p) + gain + 0.3 * noise(rng);
    if (std::fabs(off) > 17e6) v -= (std::fabs(off) - 17e6) / 1e6 * 6;
    db[i] = static_cast<float>(v);
  }
  return db;
}

// What the radio's sweep does (radio.cpp: handle_sweep), pass after pass of the radio's own plan (Sweep), each step's
// +-16.5 MHz into the map.
static BandMap swept(const std::vector<Signal> &air, int passes, unsigned seed = 3) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> gain(-6, 6);
  BandMap map(Sweep::kBandLo, Sweep::kBandHi, Sweep::kBandBin);
  Sweep sweep;
  sweep.start(5635e6, 5955e6);
  int64_t t = 1000;
  for (int pass = 0; pass < passes; pass++, sweep.pass_done()) {
    for (double c : sweep.pass()) {
      const float g = gain(rng);
      map.add(step_db(air, c, g, rng), kFs, c, kUse, t);
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

// What a scan step sees cut by an edge.
static std::vector<CutSignal> cut_in(const std::vector<float> &db) { return find_step_signals(db, kFs, kUse).cut; }

// A scan step's signals (radio.cpp: handle_scan): find_step_signals finds the ones it sees whole, and those an edge
// of its +-kEdge cuts. F2 (5760 MHz, 18 MHz wide) halfway between the 5750 and 5770 steps is cut in both: by
// the 5750 one's top edge, then by the 5770 one's bottom edge, and neither sees it whole.
static void a_signal_between_two_steps_is_cut_in_both() {
  std::mt19937 rng(5);
  const std::vector<Signal> f2 = {{5760, 18, 30, true}};
  StepSignals s = find_step_signals(step_db(f2, 5750e6, 4, rng), kFs, kUse);
  CHECK(s.whole.empty() && s.cut.size() == 1);
  auto cut = s.cut;
  if (cut.size() == 1) {
    CHECK(cut[0].cut_hi);
    CHECK(std::fabs(cut[0].f0_hz - 1e6) < 0.3e6 && cut[0].f1_hz == kUse);  // its bottom end, 5751 MHz, is seen
    CHECK(std::fabs(cut[0].snr_db - 30) < 1.5);
  }
  s = find_step_signals(step_db(f2, 5770e6, -5, rng), kFs, kUse);
  CHECK(s.whole.empty() && s.cut.size() == 1);
  cut = s.cut;
  if (cut.size() == 1) {
    CHECK(!cut[0].cut_hi);
    CHECK(cut[0].f0_hz == -kUse && std::fabs(cut[0].f1_hz + 1e6) < 0.3e6);  // its top end, 5769 MHz
    CHECK(std::fabs(cut[0].snr_db - 30) < 1.5);
  }
  // an FM video hump (soft sides: 6 dB over the floor from about 5752 to 5776 MHz) is cut the same way
  cut = cut_in(step_db({{5764, 16, 20, false}}, 5750e6, 0, rng));
  CHECK(cut.size() == 1);
  if (cut.size() == 1) CHECK(cut[0].cut_hi && cut[0].f0_hz > 1e6 && cut[0].f0_hz < 3.5e6);
}

// Whole inside a step is only a hump. A signal wider than the step (an 80 MHz Wi-Fi channel) leaves nothing over
// the floor, and a sliver of one inside an edge (under kHumpMinHz) is not listed, nor is the noise; one cut at each
// edge of a step, both.
static void cut_signals_whole_wide_and_slivers() {
  std::mt19937 rng(6);
  StepSignals s = find_step_signals(step_db({{5769, 18, 35, true}}, 5770e6, 0, rng), kFs, kUse);  // R4
  CHECK(s.whole.size() == 1 && s.cut.empty());
  s = find_step_signals(step_db({{5775, 80, 15, true}}, 5770e6, 0, rng), kFs, kUse);
  CHECK(s.whole.empty() && s.cut.empty());
  CHECK(cut_in(step_db({{5785, 18, 30, true}}, 5760e6, 0, rng)).empty());  // 5776..: 0.5 MHz in
  CHECK(cut_in(step_db({{5783, 18, 30, true}}, 5760e6, 0, rng)).empty());  // 5774..: 2.5 MHz in
  CHECK(cut_in(step_db({}, 5760e6, 3, rng)).empty());
  const auto cut = cut_in(step_db({{5750, 18, 25, true}, {5790, 18, 12, true}}, 5770e6, 0, rng));
  CHECK(cut.size() == 2);
  if (cut.size() == 2) {
    CHECK(!cut[0].cut_hi && std::fabs(cut[0].f1_hz + 11e6) < 0.3e6);  // 5741..5759: its top end
    CHECK(cut[1].cut_hi && std::fabs(cut[1].f0_hz - 11e6) < 0.3e6);   // 5781..5799: its bottom end
    CHECK(std::fabs(cut[0].snr_db - 25) < 1.5 && std::fabs(cut[1].snr_db - 12) < 1.5);
  }
}

// The sweep's plan: steps 30 MHz apart on the 5 MHz grid across the whole span, every other pass on a grid half a
// step over, a new sweep from the first grid; the span kept inside the band picture, and a span of nothing still a
// step (the DSP thread retunes to the first at once).
static void the_sweep_covers_its_span_on_two_grids() {
  Sweep s;
  s.start(5955e6, 5635e6);  // either order
  CHECK(s.lo() == 5635e6 && s.hi() == 5955e6);
  const auto a = s.pass();
  s.pass_done();
  const auto b = s.pass();
  CHECK(s.passes() == 1);
  for (const auto *p : {&a, &b}) {
    CHECK(!p->empty());
    // each end within a half step of a centre (the 5 MHz grid may move a centre 2.5 MHz)
    CHECK(p->front() - Sweep::kStep / 2 <= s.lo() + 2.5e6 && p->back() + Sweep::kStep / 2 >= s.hi());
    for (size_t i = 0; i < p->size(); i++) {
      CHECK(std::fmod((*p)[i], 5e6) == 0);
      if (i) CHECK((*p)[i] - (*p)[i - 1] == Sweep::kStep);
    }
  }
  CHECK(std::fabs(std::fabs(a.front() - b.front()) - Sweep::kStep / 2) <= 2.5e6);
  s.pass_done();
  CHECK(s.pass() == a);
  s.start(5800e6, 5800e6);
  CHECK(s.passes() == 0 && s.pass().size() == 1);
  s.start(5000e6, 7000e6);
  CHECK(s.lo() == Sweep::kBandLo && s.hi() == Sweep::kBandHi);
}

int main() {
  the_sweep_covers_its_span_on_two_grids();
  the_floor_is_flat_and_empty();
  signals_are_found_where_and_as_wide_as_they_are();
  a_signal_wider_than_a_step_is_found_whole();
  forget_and_bridge();
  a_signal_between_two_steps_is_cut_in_both();
  cut_signals_whole_wide_and_slivers();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
