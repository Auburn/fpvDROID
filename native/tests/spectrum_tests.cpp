// kp_spectrum_tests: the Spectrum tab's sweep plan (Sweep) and band picture (BandMap, find_carriers) from sweep steps
// made here, and the FFT under every spectrum (ctest: "spectrum"). Plain checks, as in radio_tests.cpp.
//
//   kp_spectrum_tests          every check; exits 1 if any fails
//   kp_spectrum_tests --time   what an FFT and a sweep step's spectrum cost, planned (Fft) and as fft_inplace was
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

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

// What the radio's sweep does (radio.cpp: handle_sweep), pass after pass of the radio's own plan (Sweep), each step
// a 40 MS/s spectrum of 1024 bins (fftshifted, dB) with a gain of its own (the chip's AGC), the RX filter rolling
// off past 17 MHz, and noise. Its +-16.5 MHz (radio.cpp's kEdge) go into the map.
static BandMap swept(const std::vector<Signal> &air, int passes, unsigned seed = 3) {
  constexpr double kFs = 40e6, kUse = 16.5e6;
  constexpr int kN = 1024;
  std::mt19937 rng(seed);
  std::normal_distribution<float> noise(0, 1);
  std::uniform_real_distribution<float> gain(-6, 6);
  BandMap map(Sweep::kBandLo, Sweep::kBandHi, Sweep::kBandBin);
  Sweep sweep;
  sweep.start(5635e6, 5955e6);
  int64_t t = 1000;
  for (int pass = 0; pass < passes; pass++, sweep.pass_done()) {
    for (double c : sweep.pass()) {
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

// ---------------------------------------------------------------------------------------- the FFT
// fft_inplace as it was before Fft: its twiddles by recurrence, as it went (Psd ran on it, and the digital timing on a
// planned copy of its own). Kept here to show that Fft changes nothing but the rounding.
static void fft_before(cf *x, int n) {
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(x[i], x[j]);
  }
  for (int len = 2; len <= n; len <<= 1) {
    const double a = -2 * M_PI / len;
    const cf wl(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
    for (int i = 0; i < n; i += len) {
      cf w(1, 0);
      for (int k = 0; k < len / 2; k++) {
        const cf u = x[i + k], v = x[i + k + len / 2] * w;
        x[i + k] = u + v;
        x[i + k + len / 2] = u - v;
        w *= wl;
      }
    }
  }
}

// The largest difference between a and b, over the largest |b|.
template <class A, class B>
static double rel_err(const A &a, const B &b) {
  double e = 0, top = 0;
  for (size_t i = 0; i < b.size(); i++) {
    e = std::max(e, std::abs(std::complex<double>(a[i]) - std::complex<double>(b[i])));
    top = std::max(top, std::abs(std::complex<double>(b[i])));
  }
  return e / top;
}

// The planned FFT gives what a DFT worked out in double gives, to within float rounding, windowed or not, and so what
// fft_inplace gave before (and gives now: it runs a plan of its own).
static void the_fft_is_the_dft() {
  std::mt19937 rng(5);
  std::normal_distribution<float> nd(0, 300);
  for (int n : {2, 8, 256, 1024}) {
    std::vector<cf> x(n);
    for (auto &v : x) v = cf(nd(rng), nd(rng));
    const std::vector<float> w = hann(n);
    std::vector<std::complex<double>> dft(n), dft_w(n);
    for (int k = 0; k < n; k++)
      for (int i = 0; i < n; i++) {
        const std::complex<double> e = std::polar(1.0, -2 * M_PI * static_cast<double>(k) * i / n);
        dft[k] += std::complex<double>(x[i]) * e;
        dft_w[k] += std::complex<double>(x[i]) * static_cast<double>(w[i]) * e;
      }
    const Fft fft(n);
    std::vector<cf> planned = x, before = x, windowed(n), inplace = x;
    fft.run(planned.data());
    fft.run(x.data(), w.data(), windowed.data());
    fft_before(before.data(), n);
    fft_inplace(inplace.data(), n);
    std::printf("FFT %4d: planned %.1e, windowed %.1e, before %.1e off the DFT; planned %.1e off before\n", n,
                rel_err(planned, dft), rel_err(windowed, dft_w), rel_err(before, dft), rel_err(planned, before));
    CHECK(rel_err(planned, dft) < 1e-6);
    CHECK(rel_err(windowed, dft_w) < 1e-6);
    CHECK(rel_err(planned, before) < 1e-5);
    CHECK(rel_err(inplace, planned) == 0);
  }
}

template <class F>
static double us_per(int runs, F f) {
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < runs; i++) f();
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / runs;
}

// What one FFT costs, planned and as before, and a sweep step's spectrum (Psd: 12 ms at 40 MS/s, 1024 points, half
// overlapping) with the planned one.
static void time_it() {
  std::mt19937 rng(5);
  std::normal_distribution<float> nd(0, 300);
  for (int n : {256, 1024}) {
    const int runs = 4000000 / n;
    std::vector<cf> x(n), y(n);
    for (auto &v : x) v = cf(nd(rng), nd(rng));
    const Fft fft(n);
    const std::vector<float> w = hann(n);
    const double planned = us_per(runs, [&] { fft.run(x.data(), w.data(), y.data()); });
    const double before = us_per(runs, [&] {
      for (int i = 0; i < n; i++) y[i] = x[i] * w[i];
      fft_before(y.data(), n);
    });
    std::printf("FFT of %4d points, windowed: planned %.2f us, before %.2f us (%.2fx)\n", n, planned, before,
                before / planned);
  }
  std::vector<cf> step(480000);
  for (auto &v : step) v = cf(nd(rng), nd(rng));
  Psd psd(1024);
  const double ms = us_per(50, [&] {
    psd.reset();
    psd.add(step.data(), step.size());
  }) / 1000;
  std::printf("a sweep step's spectrum (%d FFTs of 1024): %.2f ms\n", psd.frames(), ms);
}

int main(int argc, char **argv) {
  if (argc > 1 && std::strcmp(argv[1], "--time") == 0) {
    time_it();
    return 0;
  }
  the_fft_is_the_dft();
  the_sweep_covers_its_span_on_two_grids();
  the_floor_is_flat_and_empty();
  signals_are_found_where_and_as_wide_as_they_are();
  a_signal_wider_than_a_step_is_found_whole();
  forget_and_bridge();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
