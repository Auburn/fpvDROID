// fpvDROID decoder: how noisy the sync is. The samples of each line's back porch about their own mean (flat in
// every kind of line; the burst does not get through the sync path), mean square, and of the last kLines lines'
// the one in the middle (a jump of the levels inside a few lines' porches is not noise: one transmitter jumps 14
// times a second with a light flashing in the picture). Against the sync depth that is the sync's snr, which two
// things go by: the colour (burst_lock.h: a burst as noisy as that is not one to follow line by line) and the
// front end's channel filter (channel_filter.h). Each by a NoisyLatch of its own.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "rings.h"

namespace kp {

class SyncNoise {
public:
  static constexpr int kLines = 32;
  // The lock is lost: the measure starts over.
  void restart() { _n = 0; }
  // The line that began at ls has ended: its back porch is measured. False: it is not in the ring.
  bool line(const SyncSamples &s, double ls, double fs) {
    const double us = 1e-6 * fs;
    const int64_t i0 = static_cast<int64_t>(std::llround(ls + 5.6 * us)), i1 = static_cast<int64_t>(std::llround(ls + 8.6 * us));
    if (!s.has(i0, i1)) return false;
    float sum = 0, ss = 0;
    for (int64_t i = i0; i < i1; i++) {
      const float v = s[i];
      sum += v;
      ss += v * v;
    }
    const float cnt = static_cast<float>(i1 - i0);
    _var[_at] = std::max(0.f, ss / cnt - (sum / cnt) * (sum / cnt));
    _at = (_at + 1) % kLines;
    _n = std::min(_n + 1, kLines);
    return true;
  }
  // Another kLines lines are in since this was last true: time to judge by snr().
  bool round() const { return _at == 0; }
  // The sync depth over the noise; 0: not known (fewer than kLines lines measured, or no depth worth the name).
  float snr(float depth) const {
    if (_n < kLines || !(depth > 0.05f)) return 0;
    float v[kLines];
    std::copy(_var, _var + kLines, v);
    std::nth_element(v, v + kLines / 2, v + kLines);
    return depth / std::sqrt(std::max(v[kLines / 2], 1e-6f));
  }

private:
  float _var[kLines] = {};
  int _n = 0, _at = 0;
};

// Is the sync noisy? Not by the moment (the transmitter whose levels jump with a flashing light has its sync
// noisy for a millisecond at every jump): by how much of the time lately. Noisy, once the snr has been under `on`
// for more than half of it; clean again, once it has been over `off` for four fifths.
struct NoisyLatch {
  float on, off;
  bool noisy = false;
  float under_on = 0, under_off = 0;  // how much of the time lately the snr was under each
  // a: the share of "lately" that this measurement stands for
  bool update(float snr, float a) {
    under_on += a * ((snr < on ? 1.f : 0.f) - under_on);
    under_off += a * ((snr < off ? 1.f : 0.f) - under_off);
    noisy = noisy ? !(under_off < 0.2f) : under_on > 0.5f;
    return noisy;
  }
  void reset() {
    noisy = false;
    under_on = under_off = 0;
  }
};

}  // namespace kp
