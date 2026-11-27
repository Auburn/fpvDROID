// fpvDROID tools: a synthetic OFDM transmitter (digital video, Wi-Fi) as 40 MS/s IQ, for kp_gen's recordings and the
// digital and Wi-Fi tests (tests/digital_tests.cpp, tests/wifi_tests.cpp). Ported from FPVDesktopWiFiReceiver's
// test/digital_scan_test.cpp (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver, GPL-2.0).
//
// These are models, not captures. HDZero is continuous, flat OFDM 27 (or 17) MHz wide because that is what is known
// of it (the DM5680's datasheet: OFDM; 27 MHz per Divimath). DJI's TDD frame is upstream's measurement of a real DJI
// transmitter (20 MHz on R2). Wi-Fi is OFDM in 0.3 ms bursts with random gaps between them or, with Wi-Fi's own
// subcarriers and symbols, on a schedule of its own: an AP's beacons, a network in use (beacons(), traffic()).
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

#include "spectrum.h"  // fft_inplace

namespace kp::tx {

constexpr double kFs = 40e6;

// When a transmitter sends, by the time on its own clock (s).
using Schedule = std::function<bool(double t_s)>;

struct Ofdm {
  double offset_hz = 0;    // its centre, from the samples' centre
  double width_hz = 27e6;  // what it takes
  double snr_db = 15;      // its power density over the noise's (in its middle, with tilt_db)
  double tilt_db = 0;      // its upper edge this much over its lower edge (the shoulders of a strong signal fall away)
  double duty = 1;         // the fraction of the time it sends, in 0.3 ms bursts at random (Wi-Fi); 1: continuous
  int dji = 0;             // DJI's frame instead of duty: 1 with video, 2 with no goggles linked
  bool wifi = false;       // Wi-Fi's subcarriers and symbols (802.11a/n/ac): 312.5 kHz apart, 4 us each (3.2 and a
                           // guard of 0.8); otherwise 39 kHz apart, 28.8 us
  // In place of duty and dji: a symbol goes out when on() says so at its start. x[0] is then at t0_s on its clock and
  // each symbol's bits come from seed and the symbol's number, so that two captures of the same air agree (two steps
  // of a sweep, two pieces of a long recording).
  Schedule on;
  double t0_s = 0;
  uint64_t seed = 1;
};

// The same 64 bits for the same x, every time (splitmix64).
inline uint64_t mix(uint64_t x) {
  x += 0x9e3779b97f4a7c15ull;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}
// ... as a number in 0..1, for seed and n.
inline double unit(uint64_t seed, uint64_t n) {
  return static_cast<double>(mix(seed * 0x100000001b3ull + n) >> 11) / 9007199254740992.0;
}

// Schedules. A Wi-Fi AP with no traffic: a beacon of 0.4 ms every 102.4 ms, phase_s into it.
inline Schedule beacons(double phase_s) {
  return [phase_s](double t) {
    const double p = std::fmod(t - phase_s, 102.4e-3);
    return (p < 0 ? p + 102.4e-3 : p) < 0.4e-3;
  };
}
// A network in use: on in duty of its 0.5 ms frames at random (the same frames every time), besides its beacons.
inline Schedule traffic(double duty, uint64_t seed) {
  const Schedule b = beacons(static_cast<double>(seed % 1000) * 1e-4);
  return [duty, seed, b](double t) { return unit(seed, static_cast<uint64_t>(t / 0.5e-3)) < duty || b(t); };
}
// Bursts of 0.1 ms at random, duty of the time.
inline Schedule crackle(double duty, uint64_t seed) {
  return [duty, seed](double t) { return unit(seed, static_cast<uint64_t>(t / 0.1e-3)) < duty; };
}

// DJI's frames as measured upstream on a real transmitter (20 MHz on R2), 2.5 ms long. With video
// (rec-20261122-143904-R2): 1.73 ms sending, a gap of 0.25 ms, 0.37 ms sending, a gap of 0.15 ms. With no goggles
// linked (dji2/take1-8): 0.41 ms, a gap of 0.08 ms, 0.43 ms, a gap of 1.53 ms.
inline bool dji_on(int mode, double sample) {
  const double t = std::fmod(sample / kFs, 2.5e-3);
  if (mode == 2) return t < 0.41e-3 || (t >= 0.49e-3 && t < 0.92e-3);
  return t < 1.73e-3 || (t >= 1.98e-3 && t < 2.35e-3);
}

// Adds s to x (40 MS/s), with noise of noise_sigma (per I and Q) in mind for its level: QPSK on every subcarrier in its
// width but the centre one, a cyclic prefix. Subcarriers past +-18 MHz of the samples' centre are left out (the chip's
// RX filter). With no schedule (Ofdm::on), a frame starts at a random point and the bits are rng's.
inline void add_ofdm(std::vector<cf> &x, const Ofdm &s, double noise_sigma, std::mt19937 &rng) {
  const int N = s.wifi ? 128 : 1024, CP = s.wifi ? 32 : 128;
  const double df = kFs / N;
  const double n0 = noise_sigma * noise_sigma * 2 / kFs;  // the noise's density (I and Q)
  // each of the receiver's bins the signal has a subcarrier on (its k on k + offset / df: a signal of any width, not
  // only of N bins), and that subcarrier's amplitude (its power in time is |X_k|^2 / N^2 a sample; its density, that
  // over df)
  std::vector<int> bins;
  std::vector<float> amps;
  const long shift = std::lround(s.offset_hz / df);
  for (int bin = -N / 2; bin < N / 2; bin++) {
    const long k = bin - shift;
    const double f = s.offset_hz + k * df;
    if (k == 0 || std::fabs(k * df) > s.width_hz / 2 || std::fabs(f) > 18e6) continue;
    const double psd = n0 * std::pow(10.0, (s.snr_db + s.tilt_db * k * df / s.width_hz) / 10);
    bins.push_back((bin + N) % N);
    amps.push_back(static_cast<float>(std::sqrt(psd * df) * N));
  }
  std::vector<cf> sym(N);
  // one symbol of QPSK from bit(i) (i: 0, 1 for the first subcarrier's I and Q, ...), its samples from x[pos] on
  const auto put = [&](long long pos, auto &&bit) {
    std::fill(sym.begin(), sym.end(), cf(0, 0));
    for (size_t i = 0; i < bins.size(); i++) {
      const float re = bit(2 * i) ? amps[i] : -amps[i], im = bit(2 * i + 1) ? amps[i] : -amps[i];
      sym[bins[i]] = cf(re, im) * static_cast<float>(M_SQRT1_2);
    }
    // inverse FFT by the forward one, conjugated
    for (auto &v : sym) v = std::conj(v);
    fft_inplace(sym.data(), N);
    for (auto &v : sym) v = std::conj(v) * (1.f / N);
    for (int i = 0; i < N + CP; i++)
      if (pos + i >= 0 && pos + i < static_cast<long long>(x.size())) x[pos + i] += sym[(i - CP + N) % N];
  };
  if (s.on) {
    const double sym_s = (N + CP) / kFs;
    for (long long j = static_cast<long long>(std::floor(s.t0_s / sym_s));; j++) {
      const double start = static_cast<double>(j) * sym_s;
      const long long pos = std::llround((start - s.t0_s) * kFs);
      if (pos >= static_cast<long long>(x.size())) break;
      if (!s.on(start)) continue;
      const uint64_t key = mix(s.seed ^ mix(static_cast<uint64_t>(j)));
      put(pos, [&](size_t i) { return (mix(key + i) & 1) != 0; });
    }
    return;
  }
  std::uniform_int_distribution<int> bit(0, 1);
  const double burst = 0.3e-3 * kFs;
  double t = std::uniform_real_distribution<double>(0, s.dji ? 2.5e-3 * kFs : burst / std::max(0.01, s.duty))(rng);
  // in bursts (Wi-Fi): 0.3 ms each, apart at random (exponential), no rhythm; on average duty of the time sending
  std::exponential_distribution<double> gap(s.duty / (burst * (1 - std::min(0.99, s.duty))));
  double burst_end = -1, next_burst = t;
  for (size_t pos = 0; pos < x.size(); pos += N + CP) {
    bool on;
    if (s.dji) on = dji_on(s.dji, t + pos);
    else if (s.duty >= 1) on = true;
    else {
      if (pos >= next_burst) {
        burst_end = next_burst + burst;
        next_burst = burst_end + gap(rng);
      }
      on = pos < burst_end;
    }
    if (!on) continue;
    put(static_cast<long long>(pos), [&](size_t) { return bit(rng) != 0; });
  }
}

}  // namespace kp::tx
