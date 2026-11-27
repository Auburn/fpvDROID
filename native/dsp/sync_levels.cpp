// fpvDROID decoder: the sync levels. See sync_levels.h.
#include "sync_levels.h"

#include <algorithm>
#include <cmath>

namespace kp {

SyncLevels::SyncLevels(double fs) : _fs(fs), _hist(800, 0) {}

bool SyncLevels::mean(const SampleRings &R, double from, double to, float &out) const {
  const int64_t i0 = static_cast<int64_t>(from), i1 = static_cast<int64_t>(to);
  if (!R.has(i0, i1)) return false;
  float s = 0;
  for (int64_t i = i0; i < i1; i++) s += R.luma[i & R.mask];
  out = sign * s / static_cast<float>(i1 - i0);
  return true;
}

bool SyncLevels::survey(const std::vector<float> &buf, size_t cnt, float sgn, float &tip_o, float &blank_o) {
  std::fill(_hist.begin(), _hist.end(), 0);
  const int nb = static_cast<int>(_hist.size());
  uint32_t total = 0;
  for (size_t i = 0; i < cnt; i += 4) {
    const float v = sgn * buf[i];
    const int b = static_cast<int>((v - _hlo) / (_hhi - _hlo) * nb);
    if (b < 0 || b >= nb) continue;
    _hist[b]++;
    total++;
  }
  const bool ok = from_hist(total, tip_o, blank_o);
  std::fill(_hist.begin(), _hist.end(), 0);
  return ok;
}

void SyncLevels::acquired(float sgn, float tip_a, float blank_a) {
  sign = sgn;
  tip = tip_a;
  blank = blank_a;
  thr = 0.5f * (tip + blank);
}

void SyncLevels::set(float tip_m, float blank_m) {
  tip = tip_m;
  blank = blank_m;
  thr = 0.5f * (tip + blank);
  _gated = std::max(_gated, kSettle);
}

void SyncLevels::restart() {
  _gated = 0;
  _run = _gap = _fast = 0;
  _drift = _scatter = 0;
}

void SyncLevels::track(float m) {
  // level tracking histogram (every 8th sample), refreshed every ~40 ms
  if ((++_hdec & 7) != 0) return;
  const float v = sign * m;
  const int nb = static_cast<int>(_hist.size());
  const int b = static_cast<int>((v - _hlo) / (_hhi - _hlo) * nb);
  if (b >= 0 && b < nb) {
    _hist[b]++;
    _hist_count++;
  }
  if (_hist_count >= static_cast<uint64_t>(_fs * 0.04 / 8)) {
    float t, bl;
    if (from_hist(static_cast<uint32_t>(_hist_count), t, bl)) {
      tip = 0.8f * tip + 0.2f * t;
      blank = 0.8f * blank + 0.2f * bl;
      thr = 0.5f * (tip + blank);
    }
    std::fill(_hist.begin(), _hist.end(), 0);
    _hist_count = 0;
  }
}

// The two levels in the histogram: its lowest peak, and the next one far enough above it.
bool SyncLevels::from_hist(uint32_t total, float &tip_o, float &blank_o) {
  const int nb = static_cast<int>(_hist.size());
  if (total < 1000) return false;
  uint32_t c = 0;
  int p0 = 0, p1 = nb - 1;
  for (int i = 0; i < nb; i++) {
    c += _hist[i];
    if (c >= total * 0.005) { p0 = i; break; }
  }
  c = 0;
  for (int i = nb - 1; i >= 0; i--) {
    c += _hist[i];
    if (c >= total * 0.005) { p1 = i; break; }
  }
  if (p1 - p0 < 20) return false;
  std::vector<float> sm(nb, 0);
  for (int i = 0; i < nb; i++) {
    float s = 0;
    for (int k = -4; k <= 4; k++) {
      const int j = i + k;
      if (j >= 0 && j < nb) s += _hist[j];
    }
    sm[i] = s / 9;
  }
  float mx = 0;
  for (int i = p0; i <= p1; i++) mx = std::max(mx, sm[i]);
  std::vector<int> peaks;
  const int dist = std::max(3, (p1 - p0) / 40);
  for (int i = p0; i <= p1; i++) {
    if (sm[i] < mx * 0.02) continue;
    bool ok = true;
    for (int k = -dist; k <= dist && ok; k++) {
      const int j = i + k;
      if (k != 0 && j >= 0 && j < nb && sm[j] > sm[i]) ok = false;
    }
    if (ok && (peaks.empty() || i - peaks.back() > dist)) peaks.push_back(i);
  }
  if (peaks.size() < 2) return false;
  const float bw = static_cast<float>((_hhi - _hlo) / nb);
  const float tipv = static_cast<float>(_hlo) + (peaks[0] + 0.5f) * bw;
  const float span = (p1 - p0) * bw;
  for (size_t k = 1; k < peaks.size(); k++) {
    const float v = static_cast<float>(_hlo) + (peaks[k] + 0.5f) * bw;
    if (v - tipv > 0.12f * span) {
      tip_o = tipv;
      blank_o = v;
      return true;
    }
  }
  return false;
}

// The lines lately were off the kept levels one way: by more than three times what their scatter makes of an
// average over 8 lines (its variance a 15th of theirs), and by more than a hundredth of the sync depth. And they
// agree with each other to a tenth of the depth: in heavy noise, with clicks in the sync, two lines off the same
// way are not a slide, and levels that followed them a quarter of the way would wander.
bool SyncLevels::sliding() const {
  const float d = std::fabs(_drift), depth = blank - tip;
  return d > 0.01f * depth && d * d > 9.f / 15 * _scatter && _scatter < 0.01f * depth * depth;
}

void SyncLevels::measure(const SampleRings &R, double ls, bool slides) {
  const double us = 1e-6 * _fs;
  float t, b;
  if (!mean(R, ls + kTipUs[0] * us, ls + kTipUs[2] * us, t)) return;
  if (!mean(R, ls + kPorchUs[0] * us, ls + kPorchUs[2] * us, b)) return;
  if (b - t < 0.05f) return;
  const float et = t - tip, eb = b - blank, e = 0.5f * (et + eb);
  const float a = !settled() ? 0.1f : _fast > 0 || (slides && sliding()) ? 0.25f : 1.f / 64;
  if (_fast > 0) _fast--;
  tip += a * et;
  blank += a * eb;
  thr = 0.5f * (tip + blank);
  _gated++;
  // this line into what the next ones go by
  _scatter += ((e - _drift) * (e - _drift) - _scatter) / 32;
  _drift += (e - _drift) / 8;
}

LineLevels SyncLevels::look(const SampleRings &R, double ls) {
  const double us = 1e-6 * _fs;
  const float depth = blank - tip;
  float t0, t1, b0, b1, front;
  if (depth < 0.2f || !mean(R, ls - kFrontUs[0] * us, ls - kFrontUs[1] * us, front) ||
      !mean(R, ls + kTipUs[0] * us, ls + kTipUs[1] * us, t0) || !mean(R, ls + kTipUs[1] * us, ls + kTipUs[2] * us, t1) ||
      !mean(R, ls + kPorchUs[0] * us, ls + kPorchUs[1] * us, b0) ||
      !mean(R, ls + kPorchUs[1] * us, ls + kPorchUs[2] * us, b1)) {
    _run = 0;
    return {};
  }
  const float t = 0.5f * (t0 + t1), b = 0.5f * (b0 + b1);
  const bool pulse = std::fabs(t1 - t0) < 0.2f * depth && std::fabs(b1 - b0) < 0.2f * depth &&
                     std::fabs(front - b) < 0.3f * depth && std::fabs(b - t - depth) < 0.3f * depth;
  if (!pulse || std::fabs(t - tip) <= 0.25f * depth || std::fabs(b - blank) <= 0.25f * depth) {
    if (pulse || ++_gap > 4) _run = 0;
    return {};
  }
  _gap = 0;
  if (_run > 0 && std::fabs(t - _run_tip) < 0.25f * depth && std::fabs(b - _run_blank) < 0.25f * depth) {
    _run++;
    _run_tip += (t - _run_tip) / static_cast<float>(_run);
    _run_blank += (b - _run_blank) / static_cast<float>(_run);
  } else {
    _run = 1;
    _run_tip = t;
    _run_blank = b;
  }
  LineLevels line{LevelPair{t, b}, false};
  if (_run >= kJumpLines) {
    tip = _run_tip;
    blank = _run_blank;
    thr = 0.5f * (tip + blank);
    _run = 0;
    _fast = kFastLines;
    _drift = 0;
    jumps++;
    line.jumped = true;
  }
  return line;
}

}  // namespace kp
