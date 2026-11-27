// fpvDROID decoder: how noisy the sync is. The sync path's samples about their own mean where a line is flat, mean
// square: on the sync tip and on the back porch, the smaller of the two, and of the last kLines lines' (a field,
// about) the one in the middle. Noise is on both and on every line; what is not noise is on one of them or on some
// lines:
//  - the burst gets through the sync path's low-pass a little (a 13th of it at 20 MS/s): on a clean signal the
//    porch alone never read more than 36 depths of sync to one of noise, however clean. So the porch is read as
//    the means of samples half a subcarrier cycle apart, which have next to none of it.
//  - one camera's picture begins 8.7 us after the sync's edge, not the standard's 9.4, and its bright left edge
//    counted as noise: the porch is read as far as 8.0 us.
//  - one transmitter's sync tip is not flat for a third of the time (the one whose levels jump with a flashing
//    light), another's sits on the adapter's DC line for a quarter of every field, where the chip takes the
//    carrier half away and what follows the tip rings (its levels slide with the picture down the field, and the
//    DC line is 2 MHz under the carrier at 20 MS/s): hence the smaller of the two, and a field's lines.
//  - a jump of the levels inside a few lines' porches is not noise either: one transmitter jumps 14 times a second
//    with a light flashing in the picture.
// Read on the porch alone, over 32 lines, a strong signal from that camera passed for a noisy one half of the time,
// and the front end's channel filter came in and stayed: coloured specks all over a clean picture.
// Against the sync depth that is the sync's snr, which two things go by: the colour (burst_lock.h: a burst as noisy
// as that is not one to follow line by line) and the front end's channel filter (channel_filter.h). Each by a
// NoisyLatch of its own.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "rings.h"

namespace kp {

class SyncNoise {
public:
  // Of the last kLines lines' readings, the middle one; judged on every kEvery lines (and known once that many are
  // in, as before the window was a field long: waiting for a whole field put 50 ms on the picture's return after
  // the signal had been gone).
  static constexpr int kLines = 256, kEvery = 32;
  // The smaller of two readings is the smaller of the two, and the porch's pairs lose some of the noise with the
  // burst: on plain noise the sync read 1.25 times as clean as on the porch alone (clean recordings with noise
  // added, NTSC and PAL, 20 and 40 MS/s: 1.22 to 1.27). Put back, so that what goes by it (the channel filter, the
  // colour) sees the same noise as the same number as before.
  static constexpr float kScale = 1.25f * 1.25f;
  // The lock is lost: the measure starts over.
  void restart() { _n = 0; }
  // The line that began at ls has ended: its sync tip and back porch are measured (fsc: the colour subcarrier, Hz).
  // False: it is not in the ring.
  bool line(const SyncSamples &s, double ls, double fs, double fsc) {
    const double us = 1e-6 * fs;
    const int half = static_cast<int>(std::lround(fs / (2 * fsc)));
    float tip, porch;
    if (!flat(s, ls + 1.5 * us, ls + 4.2 * us, 0, tip) || !flat(s, ls + 5.6 * us, ls + 8.0 * us, half, porch)) return false;
    _var[_at] = kScale * std::min(tip, porch);
    _at = (_at + 1) % kLines;
    _n = std::min(_n + 1, kLines);
    return true;
  }
  // Another kEvery lines are in since this was last true: time to judge by snr().
  bool round() const { return _at % kEvery == 0; }
  // The sync depth over the noise; 0: not known (fewer than kEvery lines measured, or no depth worth the name).
  // window: of how many of the last lines (at most kLines). The whole field's steadies the channel filter and the
  // colour; a check that must see the noise rise within a few lines takes kEvery, and is the more easily fooled.
  float snr(float depth, int window = kLines) const {
    if (_n < kEvery || !(depth > 0.05f)) return 0;
    const int m = std::min({_n, window, kLines});
    float v[kLines];
    for (int k = 0; k < m; k++) v[k] = _var[(_at + kLines - 1 - k) % kLines];
    std::nth_element(v, v + m / 2, v + m);
    return depth / std::sqrt(std::max(v[m / 2], 1e-6f));
  }

private:
  // The mean square of the samples [from, to) about their mean; gap > 0: of the means of two samples that far apart.
  static bool flat(const SyncSamples &s, double from, double to, int gap, float &v) {
    const int64_t i0 = static_cast<int64_t>(std::llround(from)), i1 = static_cast<int64_t>(std::llround(to)) - gap;
    if (i1 <= i0 || !s.has(i0, i1 + gap)) return false;
    float sum = 0, sq = 0;
    for (int64_t i = i0; i < i1; i++) {
      const float x = gap > 0 ? 0.5f * (s[i] + s[i + gap]) : s[i];
      sum += x;
      sq += x * x;
    }
    const float cnt = static_cast<float>(i1 - i0);
    v = std::max(0.f, sq / cnt - (sum / cnt) * (sum / cnt));
    return true;
  }

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
