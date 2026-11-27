#include "step_grid.h"

#include <algorithm>
#include <cmath>

namespace kp {

namespace {
// A cell of noise is the mean of kSliceFrames FFT powers of independent frames (Gamma(16, 1/16) of its mean), whose
// 10th percentile is 0.696 of that mean: divided by it, noise reads 0 dB. (FPVDesktopWiFiReceiver's survey, whose
// burst thresholds were measured against such a floor, takes the 10th percentile of its bins' means.)
constexpr double kFloorQuantile = 0.10, kNoiseP10 = 0.696;
}  // namespace

StepGrid::StepGrid() : _fft(kN), _win(hann(kN)) {}

bool StepGrid::in_span(int k) const {
  const double f = std::fabs(offset_hz(k));
  return f <= _span && f >= kDcNotchHz;
}

void StepGrid::build(const cf *x, size_t n, double fs, double span_hz) {
  _fs = fs;
  _span = span_hz;
  _frames = n / kN;
  _slices = _frames / kSliceFrames;
  _frame.resize(_frames * kN);
  _slice.assign(_slices * kN, 0.f);
  cf buf[kN];
  for (size_t f = 0; f < _frames; f++) {
    _fft.run(x + f * kN, _win.data(), buf);
    float *p = &_frame[f * kN];
    // fftshifted: the FFT's bin k (k under kN / 2: above the centre) to k + kN / 2
    for (int k = 0; k < kN / 2; k++) p[k + kN / 2] = std::norm(buf[k]);
    for (int k = kN / 2; k < kN; k++) p[k - kN / 2] = std::norm(buf[k]);
    if (f / kSliceFrames >= _slices) continue;
    float *s = &_slice[f / kSliceFrames * kN];
    for (int k = 0; k < kN; k++) s[k] += p[k] * (1.f / kSliceFrames);
  }
  _floor = 0;
  std::vector<float> cells;
  for (int k = 0; k < kN; k++)
    if (in_span(k))
      for (size_t s = 0; s < _slices; s++) cells.push_back(_slice[s * kN + k]);
  if (cells.empty()) return;
  const size_t q = static_cast<size_t>(kFloorQuantile * static_cast<double>(cells.size()));
  std::nth_element(cells.begin(), cells.begin() + static_cast<long>(q), cells.end());
  _floor = static_cast<float>(cells[q] / kNoiseP10);
}

}  // namespace kp
