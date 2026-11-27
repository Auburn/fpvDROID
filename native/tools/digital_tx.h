// fpvDROID tools: a synthetic digital video transmitter (OFDM) as 40 MS/s IQ, for kp_gen's recordings and the digital
// tests (tests/digital_tests.cpp). Ported from FPVDesktopWiFiReceiver's test/digital_scan_test.cpp
// (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver, GPL-2.0).
//
// These are models, not captures. HDZero is continuous, flat OFDM 27 (or 17) MHz wide because that is what is known
// of it (the DM5680's datasheet: OFDM; 27 MHz per Divimath). DJI's TDD frame is upstream's measurement of a real DJI
// transmitter (20 MHz on R2). Wi-Fi is OFDM in 0.3 ms bursts with random gaps between them.
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <random>
#include <vector>

#include "spectrum.h"  // fft_inplace

namespace kp::tx {

constexpr double kFs = 40e6;

struct Ofdm {
  double offset_hz = 0;    // its centre, from the samples' centre
  double width_hz = 27e6;  // what it takes
  double snr_db = 15;      // its power density over the noise's
  double duty = 1;         // the fraction of the time it sends, in 0.3 ms bursts at random (Wi-Fi); 1: continuous
  int dji = 0;             // DJI's frame instead of duty: 1 with video, 2 with no goggles linked
};

// DJI's frames as measured upstream on a real transmitter (20 MHz on R2), 2.5 ms long. With video
// (rec-20261122-143904-R2): 1.73 ms sending, a gap of 0.25 ms, 0.37 ms sending, a gap of 0.15 ms. With no goggles
// linked (dji2/take1-8): 0.41 ms, a gap of 0.08 ms, 0.43 ms, a gap of 1.53 ms.
inline bool dji_on(int mode, double sample) {
  const double t = std::fmod(sample / kFs, 2.5e-3);
  if (mode == 2) return t < 0.41e-3 || (t >= 0.49e-3 && t < 0.92e-3);
  return t < 1.73e-3 || (t >= 1.98e-3 && t < 2.35e-3);
}

// Adds s to x (40 MS/s), with noise of noise_sigma (per I and Q) in mind for its level: a 1024-point FFT (39 kHz
// between subcarriers), a 1/8 cyclic prefix, QPSK, no centre subcarrier. Subcarriers past +-18 MHz of the samples'
// centre are left out (the chip's RX filter), and a frame starts at a random point.
inline void add_ofdm(std::vector<cf> &x, const Ofdm &s, double noise_sigma, std::mt19937 &rng) {
  constexpr int N = 1024, CP = 128;
  const double df = kFs / N;
  const double n0 = noise_sigma * noise_sigma * 2 / kFs;  // the noise's density (I and Q)
  const double psd = n0 * std::pow(10.0, s.snr_db / 10);
  // a subcarrier's power in time is |X_k|^2 / N^2 a sample; its density, that over df
  const float amp = static_cast<float>(std::sqrt(psd * df) * N);
  std::uniform_int_distribution<int> bit(0, 1);
  std::vector<cf> sym(N);
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
    std::fill(sym.begin(), sym.end(), cf(0, 0));
    for (int k = -N / 2; k < N / 2; k++) {
      const double f = s.offset_hz + k * df;
      if (k == 0 || std::fabs(k * df) > s.width_hz / 2 || std::fabs(f) > 18e6) continue;
      // the signal's subcarrier k falls on the receiver's k + offset/df
      const int bin = static_cast<int>(std::lround(f / df));
      if (bin < -N / 2 || bin >= N / 2) continue;
      const float re = bit(rng) ? amp : -amp, im = bit(rng) ? amp : -amp;
      sym[(bin + N) % N] = cf(re, im) * static_cast<float>(M_SQRT1_2);
    }
    // inverse FFT by the forward one, conjugated
    for (auto &v : sym) v = std::conj(v);
    fft_inplace(sym.data(), N);
    for (auto &v : sym) v = std::conj(v) * (1.f / N);
    for (int i = 0; i < N + CP && pos + i < x.size(); i++) x[pos + i] += sym[(i - CP + N) % N];
  }
}

}  // namespace kp::tx
