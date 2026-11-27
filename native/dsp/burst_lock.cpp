// fpvDROID decoder: the colour reference. See burst_lock.h.
#include "burst_lock.h"

#include <algorithm>
#include <cmath>

namespace kp {

using cf = std::complex<float>;

bool BurstLock::line(const SampleRings &R, double ls, int64_t li, float depth, bool &pal_v) {
  const double us = 1e-6 * _fs;
  const int64_t i0 = static_cast<int64_t>(ls + 6.0 * us), i1 = static_cast<int64_t>(ls + 7.8 * us);
  if (!R.has(i0, i1)) return false;
  cf b{};
  for (int64_t i = i0; i < i1; i++) b += R.chroma[i & R.mask];
  b /= static_cast<float>(i1 - i0);
  const float mag = std::abs(b);
  const double pred = _last_line < 0 ? std::arg(b) : _phase + _dphi * (li - _last_line);
  const Way now = way();
  const bool mean = now == Way::mean;
  if (mean != _mean) {
    _mean = mean;
    _z = cf{};
    _zn = 0;
    _zmag = _nvar = 0;
  }
  pal_v = false;
  if (mean) {
    // (how far the burst's samples lie from their mean: the colour path's noise, a sample)
    float nv = 0;
    for (int64_t i = i0; i < i1; i++) nv += std::norm(R.chroma[i & R.mask] - b);
    return follow_mean(b, mag, nv / static_cast<float>(i1 - i0), pred, li);
  }
  double e = std::arg(b * std::polar(1.f, static_cast<float>(-pred)));
  if (_pal) {
    // The burst sits 45 degrees either side of the reference: at baseband a normal line's burst (135 deg
    // in U/V) lands at -45, a V-switched line's (225 deg) at +45.
    pal_v = e > 0;
    e += pal_v ? -M_PI / 4 : M_PI / 4;
  }
  if (_last_line < 0) {
    _phase = pred + e;
    _dphi = 0;
  } else {
    _phase = pred + 0.2 * e;
    _dphi += 0.01 * e / std::max<int64_t>(1, li - _last_line);
  }
  _phase = std::remainder(_phase, 2 * M_PI);
  note(li, mag, static_cast<float>(std::fabs(e)));
  _size = _mag;
  // a burst near the expected size (~1/4 of the sync depth) and steady, in a signal whose sync is not too noisy
  return killer(now == Way::each && _mag > 0.08f * depth && _err < 0.5f);
}

bool BurstLock::follow_mean(cf b, float mag, float nv, double pred, int64_t li) {
  const cf d = b * std::polar(1.f, static_cast<float>(-pred));
  _zn = std::min(_zn + 1, kLines);
  const float a = 1.f / static_cast<float>(_zn);
  _z += a * (d - _z);
  _zmag += a * (mag - _zmag);
  _nvar += a * (nv - _nvar);
  const double ez = std::arg(_z), step = kStep * ez;
  _dphi += kDrift * ez / static_cast<double>(std::max<int64_t>(1, li - _last_line));
  _phase = std::remainder(pred + step, 2 * M_PI);
  _z *= std::polar(1.f, static_cast<float>(-step));  // (the mean is against the reference, and turns with it)
  note(li, mag, std::fabs(std::arg(d)));
  if (_zn < kMin) return _on;  // (as it was, until there is a mean to go by)
  const float coh = _zmag > 0 ? std::min(1.f, std::abs(_z) / _zmag) : 0;  // how much of the bursts is left in it
  const bool on = killer(coh > (_on ? kOff : kOn));
  // the burst's size as the mean has it, and the saturation down by the signal's share (LineInfo::mag divides)
  const float s = std::norm(_z), share = s / std::max(s + kNoise * _nvar, 1e-12f);
  _size = std::abs(_z) / std::max(share, 0.02f);
  return on;
}

// A burst has been followed: e, its |phase error| against the reference as it was predicted.
void BurstLock::note(int64_t li, float mag, float e) {
  _last_line = li;
  _e = e;
  _mag += 0.05f * (mag - _mag);
  _err += 0.05f * (e - _err);
}

// The colour killer, with hysteresis: a count that a good burst raises by one (to 60 at most) and a bad one lowers
// by two. Colour comes on at 40 and goes off at 0.
bool BurstLock::killer(bool ok) {
  _good = ok ? std::min(_good + 1, 60) : std::max(_good - 2, 0);
  if (!_on && _good >= 40) _on = true;
  if (_on && _good == 0) _on = false;
  return _on;
}

}  // namespace kp
