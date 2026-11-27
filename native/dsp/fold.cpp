// fpvDROID decoder: a fold of the lines, and the hsync in it. See fold.h.
#include "fold.h"

#include <algorithm>
#include <cstdlib>

namespace kp {

void Fold::shape(size_t bins_n, double fs, int step, bool wraps) {
  _us = 1e-6 * fs;
  _step = step;
  _wraps = wraps;
  v.assign(bins_n, 0.f);
  n.clear();
  _o = wraps ? bins_n : 0;
  _ps.assign((wraps ? 3 : 1) * bins_n + 1, 0.f);
  _front[0] = bins(-0.9), _front[1] = bins(-0.3);
  _tip[0] = bins(0.8), _tip[1] = bins(4.0);
  _back[0] = bins(5.6), _back[1] = bins(8.6);
  _e_half = std::max(1, bins(0.15));
  _e_from = bins(-1.5);
  _e_to = bins(1.0);
  _e_before = bins(-0.7);
  _e_low = bins(2.1);
}

std::optional<FoldPulse> Fold::pulse(int k0, int k1) {
  const int K = static_cast<int>(v.size());
  // the sums; with a fold that wraps, of the fold laid end to end three times, so that a stretch may run over
  // either end
  const int laps = _wraps ? 3 : 1;
  if (n.empty()) {
    for (int k = 0; k < laps * K; k++) _ps[static_cast<size_t>(k + 1)] = _ps[static_cast<size_t>(k)] + v[static_cast<size_t>(k % K)];
  } else {
    for (int k = 0; k < laps * K; k++) {
      const size_t j = static_cast<size_t>(k % K);
      _ps[static_cast<size_t>(k + 1)] = _ps[static_cast<size_t>(k)] + v[j] / static_cast<float>(std::max(n[j], 1));
    }
  }
  float best = 0, lowest = 1e30f, tp = 0, bk = 0;
  int e = -1;
  for (int k = k0; k <= k1; k++) {
    float t, b;
    const float r = pulse_at(k, t, b);
    lowest = std::min(lowest, t);
    if (r > best) best = r, e = k, tp = t, bk = b;
  }
  double edge;
  if (e < 0 || best < kMinDepth || !edge_of(e, tp, bk, edge)) return std::nullopt;
  return FoldPulse{edge, tp, bk, best, lowest};
}

// How much like an hsync that begins at bin k the fold looks there: how far a stretch as long as most of a sync
// tip lies under what is before it and behind it (tip, blank: its level, and the back porch's). Under the lower
// of the two: the front porch is short, the sync path's filter runs the end of the picture into it, and a bright
// picture there must not make a place a microsecond early look the better one.
float Fold::pulse_at(int k, float &tip, float &blank) const {
  const float front = mean(k + _front[0], k + _front[1]);
  tip = mean(k + _tip[0], k + _tip[1]);
  blank = mean(k + _back[0], k + _back[1]);
  return std::min(front, blank) - tip;
}

// ... and where that hsync's leading edge is (bins, to a fraction; e: where pulse_at had it, which is anywhere
// within a microsecond). First to the bin where the fold goes under half way between the two levels, then by
// area: how much of a stretch around the edge is up at blanking's level (the back porch's: the longer and the
// cleaner of the two), which takes every sample of the edge into the count. That edge alone, against the tip's
// level over the pulse's first 2 us: the equalising and the broad pulses of the vertical interval are in the
// fold with the hsyncs, and they begin as an hsync does and end otherwise.
bool Fold::edge_of(int e, float tip, float blank, double &edge) const {
  const float mid = 0.5f * (tip + blank);
  const int w = _e_half;
  int c = e + _e_from;
  const int end = e + _e_to;
  while (c < end && !(mean(c, c + w) < mid && mean(c + w, c + 2 * w) < mid)) c++;
  if (c >= end) return false;
  // (twice: in noise the fold first goes under half way a little early, and the stretch is then put again around
  // where the area has the edge)
  for (int pass = 0; pass < 2; pass++) {
    const int l0 = c + _e_before, l1 = c + _e_to;
    const float low = mean(l1, c + _e_low);
    if (blank - low < 0.5f * (blank - tip)) return false;
    edge = l0 + static_cast<double>(mean(l0, l1) - low) / (blank - low) * (l1 - l0);
    const int again = static_cast<int>(std::lround(edge));
    if (again == c || std::abs(again - c) > _e_to) break;
    c = again;
  }
  return true;
}

}  // namespace kp
