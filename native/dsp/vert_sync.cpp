// fpvDROID decoder: the vertical sync of a weak signal. See vert_sync.h.
#include "vert_sync.h"

#include <cmath>

namespace kp {

std::optional<FieldStart> VertSync::line(const SyncSamples &s, double ls, double T, double fs, float sign,
                                         LevelPair levels, bool pal, int64_t line) {
  const double us = 1e-6 * fs;
  const float depth = levels.blank - levels.tip;
  for (int half = 0; half < 2; half++) {
    const double h0 = ls + half * 0.5 * T;
    const int64_t i0 = static_cast<int64_t>(std::llround(h0 + 5.5 * us)), i1 = static_cast<int64_t>(std::llround(h0 + 26.0 * us));
    float q = 1;
    if (s.has(i0, i1) && depth > 0.02f) {
      float sum = 0;
      int c = 0;
      for (int64_t i = i0; i < i1; i += 4, c++) sum += s[i];
      q = (sign * sum / static_cast<float>(c) - levels.tip) / depth;
    }
    _q[_n++ & 15] = q;
  }
  if (_n < 12) return std::nullopt;
  // the half lines so far end with this line's second; a vertical sync that began with the line three back
  // (odd) or in its middle (even) has all of its broad pulses, and the half line after them, in by now
  const int nb = pal ? 5 : 6;
  const auto broad = [&](uint64_t back) { return _q[(_n - 1 - back) & 15] < 0.5f; };
  for (int even = 0; even < 2; even++) {
    const uint64_t first = static_cast<uint64_t>(7 - even);  // half lines back from the newest
    bool ok = !broad(first + 1) && !broad(first - static_cast<uint64_t>(nb));
    for (int k = 0; k < nb && ok; k++) ok = broad(first - static_cast<uint64_t>(k));
    if (ok) return FieldStart{even, line - 3};
  }
  return std::nullopt;
}

}  // namespace kp
