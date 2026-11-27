// fpvDROID: a sweep step's capture cut into time and frequency once, for every measure that needs to see how the band
// comes and goes within its 12 ms: the frames of 256 points (6.4 us at 40 MS/s) one after another, each bin's power in
// each frame, and in each slice of 16 frames (~0.1 ms). digital.h reads a block's power frame by frame (its duty and
// its TDD frame); a measure of bursts reads the slices, the floor of their cells and, for a slice it looks at closer,
// its frames one by one.
#pragma once
#include <cstddef>
#include <vector>

#include "spectrum.h"

namespace kp {

class StepGrid {
public:
  static constexpr int kN = 256;           // points an FFT: bins of fs / 256 (156.25 kHz at 40 MS/s)
  static constexpr int kSliceFrames = 16;  // frames a slice: ~0.1 ms at 40 MS/s

  StepGrid();
  // n samples at fs in a row, frames from x[0] on (Hann window), centred on the hardware centre; the floor from the bins
  // within +-span_hz of it, less the DC notch (kDcNotchHz). Its buffers are kept from one step to the next.
  void build(const cf *x, size_t n, double fs, double span_hz);

  double fs() const { return _fs; }
  double bin_hz() const { return _fs / kN; }
  // Bins are fftshifted: bin k is offset_hz(k) from the centre (kN / 2 on it).
  double offset_hz(int k) const { return (k - kN / 2) * bin_hz(); }
  // The bins the floor is taken from: within +-span_hz of the centre, and not in the DC notch.
  bool in_span(int k) const;
  size_t frames() const { return _frames; }
  size_t slices() const { return _slices; }  // frames() / kSliceFrames: the frames past the last whole slice are in none
  // The kN bins' power in frame f (|X_k|^2 of its windowed FFT), and in slice s (the mean of its frames,
  // s * kSliceFrames on).
  const float *frame(size_t f) const { return &_frame[f * kN]; }
  const float *slice(size_t s) const { return &_slice[s * kN]; }
  // The power of a cell (a slice's bin) of noise: the 10th percentile of the cells of the bins in the span, over that
  // percentile's share of a noise cell's mean. A low percentile of every cell, not of the bins' means: a busy signal
  // as wide as the step raises every bin's mean, but between its bursts the cells are noise. 0 under one slice.
  float floor() const { return _floor; }

private:
  Fft _fft;
  std::vector<float> _win;
  double _fs = 0, _span = 0;
  size_t _frames = 0, _slices = 0;
  std::vector<float> _frame, _slice;
  float _floor = 0;
};

}  // namespace kp
