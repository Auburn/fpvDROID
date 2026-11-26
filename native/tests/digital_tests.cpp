// kp_digital_tests: the Spectrum tab telling digital signals apart (dsp/digital.h), from sweep steps made here: 12 ms
// of 40 MS/s IQ a step, on the radio's own plan (Sweep), as radio.cpp's handle_sweep takes them (ctest: "digital").
// The cases are FPVDesktopWiFiReceiver's test/digital_scan_test.cpp (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver,
// GPL-2.0), run through the sweep's two grids instead of upstream's scan.
//
//   kp_digital_tests          every case; exits 1 if any fails
//   kp_digital_tests -v       ... with every block each step sees, and how it measures
//   kp_digital_tests --time   what the digital scan costs a sweep step, against the step's spectrum
//
// The signals are models, not captures (tools/digital_tx.h): continuous flat OFDM for HDZero, DJI's TDD frame as
// measured upstream on a real transmitter, Wi-Fi in bursts, and analog FM video.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "digital.h"
#include "digital_tx.h"
#include "log.h"
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
constexpr double kFs = 40e6;
constexpr double kEdge = 16.5e6;                                      // radio.cpp's: the span a step uses
constexpr size_t kStepSamples = static_cast<size_t>(0.012 * kFs);    // radio.cpp's kPsdSpan
constexpr double kNoiseSigma = 20;                                    // the chip's noise (12-bit counts)
constexpr int64_t kStepMs = 180;                                      // a retune and its capture (~2 s a pass)

static std::mt19937 rng(1234);

enum class Sig { Ofdm, FmBars, FmBusy };
struct Air {
  Sig type = Sig::Ofdm;
  double mhz = 0;          // its centre
  double width_mhz = 27;   // OFDM: what it takes
  double snr_db = 15;      // OFDM: density over the noise's; FM: power over the noise's in 20 MHz
  double duty = 1;         // OFDM: fraction of the time sending, in 0.3 ms bursts (Wi-Fi)
  int dji = 0;             // OFDM: DJI's frame instead (1 with video, 2 with no goggles linked)
};
static Air hdzero(double mhz, double snr = 15) { return {Sig::Ofdm, mhz, 27, snr}; }
static Air narrow(double mhz) { return {Sig::Ofdm, mhz, 17, 15}; }
static Air dji(double mhz, double snr = 15, int mode = 1) { return {Sig::Ofdm, mhz, 18, snr, 1, mode}; }
static Air wifi(double mhz, double width, double duty) { return {Sig::Ofdm, mhz, width, 20, duty}; }
static Air fm(double mhz, bool busy, double snr = 30) { return {busy ? Sig::FmBusy : Sig::FmBars, mhz, 20, snr}; }

// The chip's RX filter at 80 MS/s: a windowed sinc (Blackman) cut at 18 MHz, for analog FM made at twice the rate.
static const std::vector<float> &rx_filter() {
  static const std::vector<float> taps = [] {
    constexpr int kTaps = 63;
    std::vector<float> t(kTaps);
    double sum = 0;
    for (int i = 0; i < kTaps; i++) {
      const double m = i - (kTaps - 1) / 2.0, fc = 18e6 / (2 * kFs);
      const double sinc = m == 0 ? 2 * fc : std::sin(2 * M_PI * fc * m) / (M_PI * m);
      const double w = 0.42 - 0.5 * std::cos(2 * M_PI * i / (kTaps - 1)) + 0.08 * std::cos(4 * M_PI * i / (kTaps - 1));
      t[i] = static_cast<float>(sinc * w);
      sum += t[i];
    }
    for (auto &v : t) v = static_cast<float>(v / sum);
    return t;
  }();
  return taps;
}

// Analog FM video, NTSC: lines of 63.5 us (sync 4.7 us at -40 IRE, black 0, a picture of 52.6 us), ~8 MHz from sync tip
// to white (a typical analog transmitter), with 8 bars or a busy picture (a new level every 0.2 us). Made at 80 MS/s
// and put through the RX filter before every other sample is kept: one beside a step's edge is cut there as on the
// chip, instead of folding over to the other edge.
static void add_fm(std::vector<cf> &x, double offset, const Air &s) {
  constexpr double fs2 = 2 * kFs;
  const double line = 63.5e-6 * fs2, sync = 4.7e-6 * fs2, active0 = 10.9e-6 * fs2;
  const double dev_per_unit = 8e6 / 1.4;  // -0.4 .. 1.0
  const double p = kNoiseSigma * kNoiseSigma * 2 * 0.5 * std::pow(10.0, s.snr_db / 10);  // 20 of the 40 MHz
  const float a = static_cast<float>(std::sqrt(p));
  const auto &taps = rx_filter();
  std::vector<cf> hi(2 * x.size() + taps.size());
  std::uniform_real_distribution<double> u(0, 1);
  double ph = std::uniform_real_distribution<double>(0, 2 * M_PI)(rng), pix = 0;
  const double t0 = std::uniform_real_distribution<double>(0, line)(rng);
  for (size_t i = 0; i < hi.size(); i++) {
    const double tl = std::fmod(t0 + static_cast<double>(i), line);
    double v;
    if (tl < sync) v = -0.4;
    else if (tl < active0) v = 0;
    else if (s.type == Sig::FmBusy) {
      if (i % 16 == 0) pix = u(rng);
      v = pix;
    } else {
      v = std::floor((tl - active0) / ((line - active0) / 8)) / 7.0;
    }
    ph += 2 * M_PI * (offset + (v - 0.3) * dev_per_unit) / fs2;
    hi[i] = a * cf(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
  }
  for (size_t k = 0; k < x.size(); k++) {
    cf acc(0, 0);
    const cf *h = &hi[2 * k];
    for (size_t t = 0; t < taps.size(); t++) acc += h[t] * taps[t];
    x[k] += acc;
  }
}

// A step's samples, its hardware centre on centre_hz: noise, and whatever on the air reaches its RX filter.
// dc_notch: the chip's DC filter too (a one-pole DC block, ~0.5 MHz wide).
static std::vector<cf> step_samples(double centre_hz, const std::vector<Air> &air, bool dc_notch = false,
                                    size_t n = kStepSamples) {
  std::normal_distribution<float> nd(0, static_cast<float>(kNoiseSigma));
  std::vector<cf> x(n);
  for (auto &v : x) v = cf(nd(rng), nd(rng));
  for (const auto &s : air) {
    const double off = s.mhz * 1e6 - centre_hz;
    if (s.type == Sig::Ofdm) {
      if (std::fabs(off) - s.width_mhz * 1e6 / 2 < 18e6) {
        tx::Ofdm o;
        o.offset_hz = off;
        o.width_hz = s.width_mhz * 1e6;
        o.snr_db = s.snr_db;
        o.duty = s.duty;
        o.dji = s.dji;
        tx::add_ofdm(x, o, kNoiseSigma, rng);
      }
    } else if (std::fabs(off) < 30e6) {
      add_fm(x, off, s);
    }
  }
  if (dc_notch) {
    cf prev_in(0, 0), prev_out(0, 0);
    for (auto &v : x) {
      const cf out = v - prev_in + 0.92f * prev_out;
      prev_in = v;
      prev_out = out;
      v = out;
    }
  }
  return x;
}

static std::vector<float> spectrum_of(const std::vector<cf> &x) {
  Psd psd(1024);
  psd.add(x.data(), x.size());
  return psd.db();
}

// ---------------------------------------------------------------------------------------- the sweep
// What the radio does on the Spectrum tab (radio.cpp: handle_sweep, add_digital), pass after pass of its own plan
// over 5635..5955 MHz: each step's spectrum into the band picture and, with its IQ, into the digital scan; the
// verdicts kept, and at the end of a pass those not found again for DigitalMap::kKeepMs forgotten. A step whose span
// is nowhere near anything on the air holds only noise and is left out (it would change nothing but the time taken),
// unless every_step.
struct Swept {
  BandMap band{Sweep::kBandLo, Sweep::kBandHi, Sweep::kBandBin};
  DigitalMap digital;
  DigitalScan scan;
  Sweep sweep;
  int64_t t = 1000;

  Swept() { sweep.start(5635e6, 5955e6); }

  // One pass on the sweep's grid of the moment; what it found (the verdicts as they stand after it) is digital.all().
  void pass(const std::vector<Air> &air, bool dc_notch = false, bool every_step = false) {
    for (double c : sweep.pass()) {
      bool near = every_step;
      for (const auto &s : air) near = near || std::fabs(c - s.mhz * 1e6) < kEdge + s.width_mhz * 1e6 / 2 + 5e6;
      t += kStepMs;
      if (!near) continue;
      const auto x = step_samples(c, air, dc_notch);
      const auto db = spectrum_of(x);
      band.add(db, kFs, c, kEdge, t);
      digital.add(scan.step(db, x.data(), x.size(), kFs, c, kEdge), t);
    }
    digital.forget(sweep.lo(), sweep.hi(), t - DigitalMap::kKeepMs);
    scan.reset();
    sweep.pass_done();
  }
};

// A pass on each grid: what the Spectrum tab says after its first two passes.
static std::vector<DigitalSignal> judged(const std::vector<Air> &air, bool every_step = false) {
  Swept s;
  s.pass(air, false, every_step);
  s.pass(air, false, every_step);
  return s.digital.all();
}

static void expect(const char *name, const std::vector<DigitalSignal> &got, DigitalKind kind, double mhz = 0,
                   double min_w = 0, double max_w = 1e9) {
  bool ok;
  if (kind == DigitalKind::None) ok = got.empty();
  else
    ok = got.size() == 1 && got[0].kind == kind && std::fabs(got[0].centre_hz / 1e6 - mhz) <= 1.5 &&
         got[0].width_hz / 1e6 >= min_w && got[0].width_hz / 1e6 <= max_w;
  g_checks++;
  if (!ok) g_failed++;
  std::printf("%-46s %s", name, ok ? "ok  " : "FAIL");
  if (got.empty()) std::printf("  (nothing)");
  for (const auto &d : got)
    std::printf("  [%s %.2f MHz, %.1f MHz, %.1f dB]", digital_code(d.kind), d.centre_hz / 1e6, d.width_hz / 1e6,
                d.snr_db);
  std::printf("\n");
}

// ---------------------------------------------------------------------------------------- the tests
// The measures themselves on one stretch of samples: DJI's frame found at 2.5 ms in both of its patterns, none in
// continuous OFDM or in Wi-Fi's bursts; continuous OFDM there all the time, Wi-Fi at 30 % not.
static void the_frame_and_the_duty() {
  const double f0 = -9e6, f1 = 9e6;
  for (int mode : {1, 2}) {
    const auto x = step_samples(5770e6, {dji(5770, 15, mode)});
    const double frame = tdd_frame(x.data(), x.size(), kFs, f0, f1);
    std::printf("DJI frame (pattern %d): %.3f ms, duty %.2f\n", mode, frame * 1e3,
                band_duty(x.data(), x.size(), kFs, f0, f1));
    CHECK(std::fabs(frame - 2.5e-3) <= 0.1e-3);
  }
  auto x = step_samples(5770e6, {hdzero(5770)});
  CHECK(tdd_frame(x.data(), x.size(), kFs, -13.5e6, 13.5e6) == 0);
  CHECK(band_duty(x.data(), x.size(), kFs, -13.5e6, 13.5e6) > 0.95f);
  x = step_samples(5770e6, {wifi(5770, 16.25, 0.3)});
  CHECK(tdd_frame(x.data(), x.size(), kFs, f0, f1) == 0);
  CHECK(band_duty(x.data(), x.size(), kFs, f0, f1) < 0.9f);
}

// Upstream's cases, each a pass on each grid of the sweep.
static void the_cases() {
  using K = DigitalKind;
  const double R[] = {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917};
  // HDZero (27 MHz) on every R channel: wider than most of a step's span, it is often cut between two steps
  for (int i = 0; i < 8; i++) {
    char name[64];
    std::snprintf(name, sizeof name, "HDZero 27 MHz R%d", i + 1);
    expect(name, judged({hdzero(R[i])}), K::HDZero, R[i], 25, 29);
  }
  expect("HDZero 27 MHz E1", judged({hdzero(5705)}), K::HDZero, 5705, 25, 29);
  expect("HDZero 27 MHz F1", judged({hdzero(5740)}), K::HDZero, 5740, 25, 29);
  expect("HDZero 27 MHz R5, weak (9 dB)", judged({hdzero(5806, 9)}), K::HDZero, 5806);
  expect("HDZero 27 MHz R5, very weak (5 dB): no", judged({hdzero(5806, 5)}), K::None);
  expect("HDZero 27 MHz R4, strong (35 dB)", judged({hdzero(5769, 35)}), K::HDZero, 5769);
  // narrow (17 MHz, 540p60)
  expect("HDZero 17 MHz R4", judged({narrow(5769)}), K::HDZeroNarrow, 5769, 15, 19);
  expect("HDZero 17 MHz R7", judged({narrow(5880)}), K::HDZeroNarrow, 5880, 15, 19);
  // two HDZero at once (R2 and R6)
  {
    const auto got = judged({hdzero(5695), hdzero(5843, 12)});
    const bool ok = got.size() == 2 && got[0].kind == K::HDZero && got[1].kind == K::HDZero;
    g_checks++;
    if (!ok) g_failed++;
    std::printf("%-46s %s  (%zu signals)\n", "HDZero R2 + R6 at once", ok ? "ok  " : "FAIL", got.size());
  }
  // other digital signals: not HDZero
  expect("continuous OFDM 18 MHz at 5745 (not HDZero)", judged({{Sig::Ofdm, 5745, 18, 15}}), K::Other, 5745);
  // DJI (20 MHz, a TDD frame of 2.5 ms): even on an HDZero channel
  expect("DJI 20 MHz R2", judged({dji(5695)}), K::DJI, 5695, 16, 21);
  expect("DJI 20 MHz R2, strong (30 dB)", judged({dji(5695, 30)}), K::DJI, 5695);
  expect("DJI 20 MHz R2, weak (10 dB)", judged({dji(5695, 10)}), K::DJI, 5695);
  expect("DJI 20 MHz R2, no goggles", judged({dji(5695, 15, 2)}), K::DJI, 5695);
  expect("DJI 20 MHz R2, no goggles, strong (30 dB)", judged({dji(5695, 30, 2)}), K::DJI, 5695);
  expect("DJI 20 MHz at 5745", judged({dji(5745)}), K::DJI, 5745);
  expect("DJI 20 MHz V1/V2 CH3 (5735)", judged({dji(5735)}), K::DJI, 5735);
  expect("Wi-Fi 20 MHz in bursts (30 %) at 5745: no", judged({wifi(5745, 16.25, 0.3)}), K::None);
  expect("Wi-Fi 40 MHz continuous at 5775: no", judged({wifi(5775, 38, 1)}), K::None);
  // analog video (FM): never digital
  expect("analog FM, bars, R4, strong: no", judged({fm(5769, false)}), K::None);
  expect("analog FM, busy picture, R5: no", judged({fm(5806, true)}), K::None);
  expect("analog FM, busy picture, R3, 35 dB: no", judged({fm(5732, true, 35)}), K::None);
  expect("analog FM, busy picture, R8 (cut by a step): no", judged({fm(5917, true, 35)}), K::None);
  expect("noise only: no", judged({}, true), K::None);
}

// A 27 MHz signal on R8 is judged on one of the sweep's grids only: the other cuts it 3 MHz from a step's lower edge,
// too little of it in the step below to be joined. The verdict found on one grid holds through the passes on the
// other, and goes once the signal has gone for DigitalMap::kKeepMs.
static void a_verdict_holds_across_passes_that_cannot_judge_it() {
  const std::vector<Air> air = {hdzero(5917)};
  Swept s;
  int found_in = -1;
  for (int pass = 0; pass < 2; pass++) {
    Swept one;
    for (int k = 0; k < pass; k++) one.sweep.pass_done();  // that grid on its own
    one.pass(air);
    if (!one.digital.all().empty()) found_in = found_in < 0 ? pass : 2;
  }
  CHECK(found_in == 1);  // grid 1 only
  for (int pass = 0; pass < 6; pass++) {
    s.pass(air);
    if (pass >= 1) {
      CHECK(s.digital.all().size() == 1);
      if (!s.digital.all().empty()) CHECK(s.digital.all()[0].kind == DigitalKind::HDZero);
    }
  }
  // switched off: the verdict stays a while, then goes
  const int64_t off_at = s.t;
  int passes_left = 0;
  while (!s.digital.all().empty() && passes_left < 10) {
    s.pass({}, false, true);
    passes_left++;
  }
  std::printf("verdict gone %d passes (%.1f s) after the signal\n", passes_left, (s.t - off_at) / 1000.0);
  CHECK(s.digital.all().empty());
  CHECK(passes_left >= 3);  // not at the first pass that misses it
}

// The verdicts kept: one per signal (a new verdict within kSameHz takes the old one's place), forgotten only in the
// span asked and only when older; a carrier gets the verdict near its centre if about as wide.
static void the_map_of_verdicts() {
  DigitalMap m;
  DigitalSignal a{5695e6, 18e6, 20, DigitalKind::DJI, 0}, b{5843e6, 27e6, 15, DigitalKind::HDZero, 0};
  m.add({a, b}, 1000);
  a.centre_hz = 5696.5e6;
  m.add({a}, 3000);
  CHECK(m.all().size() == 2);
  m.forget(5600e6, 6000e6, 2000);  // b, found at 1000, goes
  CHECK(m.all().size() == 1 && m.all()[0].kind == DigitalKind::DJI && m.all()[0].seen_ms == 3000);
  m.forget(5800e6, 6000e6, 9000);  // outside the span: stays
  CHECK(m.all().size() == 1);
  const auto &v = m.all();
  CHECK(digital_for(v, Carrier{5695.0, 19, 22}) == &v[0]);
  CHECK(digital_for(v, Carrier{5700.0, 19, 22}) == &v[0]);  // 3.5 MHz from the verdict's centre
  CHECK(digital_for(v, Carrier{5701.0, 19, 22}) == nullptr);
  CHECK(digital_for(v, Carrier{5696.0, 45, 22}) == nullptr);  // run together with a neighbour
  CHECK(digital_for(v, Carrier{5696.0, 6, 22}) == nullptr);
}

// Through the band picture, as the app reads it: what is on the 5.8 GHz band, swept with the chip's DC notch, comes
// out of band_json with each carrier's kind (or none, for the analog transmitter and Wi-Fi's bursts).
struct JsonCarrier {
  double mhz, width;
  std::string kind;
};
static std::vector<JsonCarrier> carriers_in(const std::string &json) {
  std::vector<JsonCarrier> out;
  size_t at = json.find("\"carriers\":[");
  while (at != std::string::npos && (at = json.find("{\"mhz\":", at)) != std::string::npos) {
    JsonCarrier c{};
    std::sscanf(json.c_str() + at, "{\"mhz\":%lf,\"width\":%lf", &c.mhz, &c.width);
    const size_t end = json.find('}', at);
    const size_t k = json.find("\"kind\":\"", at);
    if (k != std::string::npos && k < end) c.kind = json.substr(k + 8, json.find('"', k + 8) - k - 8);
    out.push_back(c);
    at = end;
  }
  return out;
}

static const JsonCarrier *near(const std::vector<JsonCarrier> &cs, double mhz) {
  for (const auto &c : cs)
    if (std::fabs(c.mhz - mhz) < 3) return &c;
  return nullptr;
}

static void kinds_come_out_with_the_band_picture() {
  const std::vector<Air> air = {hdzero(5658), dji(5735), fm(5806, false, 25), wifi(5865, 16.25, 0.3),
                                narrow(5917)};
  Swept s;
  for (int pass = 0; pass < 4; pass++) s.pass(air, true, true);
  double first = 0;
  const auto db = s.band.get(5635e6, 5955e6, &first);
  const std::string json = band_json(db, first, s.band.bin_hz(), 0, 2 * kEdge, 4, s.digital.all());
  const auto cs = carriers_in(json);
  for (const auto &c : cs) std::printf("carrier %.1f MHz, %.1f MHz wide: %s\n", c.mhz, c.width, c.kind.empty() ? "-" : c.kind.c_str());
  const JsonCarrier *h = near(cs, 5658), *d = near(cs, 5735), *a = near(cs, 5806), *w = near(cs, 5865),
                    *n = near(cs, 5917);
  CHECK(h && h->kind == "hdzero");
  CHECK(d && d->kind == "dji");
  CHECK(a && a->kind.empty());
  CHECK(!w || w->kind.empty());  // at 30 % of the time it may not stand 6 dB over the floor at all
  CHECK(n && n->kind == "hdzero_narrow");
  // and nothing judged without verdicts: the carriers as before, no "kind" at all
  CHECK(band_json(db, first, s.band.bin_hz(), 0, 2 * kEdge, 4).find("\"kind\"") == std::string::npos);
}

// ---------------------------------------------------------------------------------------- the cost
template <class F>
static double ms_per(int runs, F f) {
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < runs; i++) f();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / runs;
}

static void time_it() {
  constexpr int kRuns = 30;
  struct Case {
    const char *name;
    std::vector<Air> air;
    double centre;
  };
  const Case cases[] = {
      {"noise only", {}, 5770e6},
      {"analog FM (R4)", {fm(5769, false)}, 5770e6},
      {"HDZero 27 MHz whole", {hdzero(5769)}, 5770e6},
      {"DJI 20 MHz whole", {dji(5770)}, 5770e6},
      {"two blocks (DJI + HDZero half)", {dji(5760), hdzero(5790)}, 5770e6},
  };
  std::printf("a sweep step: %zu samples (12 ms at 40 MS/s)\n", kStepSamples);
  for (const auto &c : cases) {
    const auto x = step_samples(c.centre, c.air);
    const auto db = spectrum_of(x);
    const double psd = ms_per(kRuns, [&] { spectrum_of(x); });
    const double scan = ms_per(kRuns, [&] {
      DigitalScan s;
      s.step(db, x.data(), x.size(), kFs, c.centre, kEdge);
    });
    // upstream's way: band_duty and tdd_frame each with FFTs of their own, for every wide block
    const double each = ms_per(kRuns, [&] {
      for (const auto &r : find_wide_regions(db, kFs, kEdge)) {
        band_duty(x.data(), x.size(), kFs, r.f0_hz, r.f1_hz);
        tdd_frame(x.data(), x.size(), kFs, r.f0_hz, r.f1_hz);
      }
    });
    std::printf("%-32s spectrum %.2f ms, digital scan %.2f ms (upstream's way %.2f ms)\n", c.name, psd, scan, each);
  }
}

int main(int argc, char **argv) {
  set_log_debug(argc > 1 && std::strcmp(argv[1], "-v") == 0);
  if (argc > 1 && std::strcmp(argv[1], "--time") == 0) {
    time_it();
    return 0;
  }
  the_frame_and_the_duty();
  the_map_of_verdicts();
  the_cases();
  a_verdict_holds_across_passes_that_cannot_judge_it();
  kinds_come_out_with_the_band_picture();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
