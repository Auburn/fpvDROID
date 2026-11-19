// From Play RX (github.com/scerstt/PlayRX, GPL-2.0, src/radio/combine.*), where it was tuned on RTL8812BU
// adapters; namespace kp here instead of arx, otherwise unchanged.
//
// Two-antenna video: path A and path B of the 8812BU (one chip, one clock, so their samples line up exactly)
// combined into one stream before FM demodulation (maximal-ratio combining).
//
// Every block of kBlock samples: each path's signal and noise power (the M2M4 estimator: the FM carrier has a
// constant envelope, so  S = sqrt(2 M2^2 - M4),  N = M2 - S), and the phase of B relative to A (from their
// cross-correlation). B is turned into step with A, and each path is weighted by its amplitude / noise
// (sqrt(S) / N), so a faded or missing antenna drops out by itself and two good ones add up (~3 dB more SNR,
// and far fewer of the deep fades that make sparkles). Weights move smoothly from block to block.
#pragma once
#include <complex>
#include <cstddef>

namespace kp {

using cf = std::complex<float>;

class Combiner {
public:
  static constexpr size_t kBlock = 1024;

  void reset();
  // a: path A, replaced by the combined stream; b: path B (DSP thread only, like take_stats).
  void process(cf *a, const cf *b, size_t n);  // in blocks of kBlock (a shorter last block is fine)

  struct Stats {
    float snr_a_db = -99, snr_b_db = -99, snr_c_db = -99;  // per path and combined (sum), wideband, dB
    float b_share = 0;      // B's part of the combined signal-to-noise (0 = A only, 0.5 = equal, 1 = B only)
    float a_fade = 0;       // fraction of the time A alone was below kFadeDb ...
    float rescued = 0;      // ... of which the combined stream was back above it
    float coherence = 0;    // |correlation| of A and B (1 = the same signal, 0 = no relation)
  };
  // averages since the last call (call ~1/s)
  Stats take_stats();
  static constexpr float kFadeDb = 6;  // wideband SNR below which the picture starts to sparkle (roughly)

private:
  void block(cf *a, const cf *b, size_t n);
  bool _init = false;
  cf _dca{}, _dcb{};
  double _m2a = 0, _m4a = 0, _m2b = 0, _m4b = 0;
  std::complex<double> _c{};
  double _noise_a = 0, _noise_b = 0;  // noise power of each path, averaged slowly
  cf _wa{1, 0}, _wb{0, 0};  // the weights in effect at the end of the last block
  // stats
  double _sa = 0, _sb = 0, _sc = 0, _share = 0, _coh = 0;
  size_t _nb = 0, _fade = 0, _resc = 0;
};

}  // namespace kp
