// fpvDROID decoder: a fold of the lines, and the hsync in it. The sync path's samples of many lines laid on top of
// each other, a mean for each place along the line: the hsync, which comes at the same place in every line, stands
// out of the noise in it as it does in no one line. The decoder folds three ways (a stretch around where each
// line's hsync is due and the whole line coarsely: line_hold.h; a buffer at acquisition: acquire.h), and each asks
// the same of its fold: is there an hsync in it, where, and at which levels.
#pragma once
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace kp {

// An hsync as a fold shows it.
struct FoldPulse {
  double edge;       // its leading edge, in bins from the fold's first, to a fraction
  float tip, blank;  // its level, and the back porch's
  float depth;       // how far it lies under what is before it and behind it
  float lowest;      // the lowest stretch as long as a sync tip, of all the places looked at
  // Is it the lowest thing in all that was looked at? In a whole line the hsync is: lower than anything in the
  // picture.
  bool lowest_there() const { return tip <= lowest + 0.25f * depth; }
};

class Fold {
public:
  // bins: how many places; fs: the sample rate; step: samples to a bin; wraps: the fold is a whole line, its last
  // bin followed by its first, and a pulse may lie across its ends.
  void shape(size_t bins, double fs, int step, bool wraps);
  std::vector<float> v;  // each place's mean: whoever folds, writes these ...
  std::vector<int> n;    // ... or its sum, of so many samples (empty: v holds the means)
  // So many microseconds, in bins
  int bins(double us) const { return static_cast<int>(std::lround(us * _us / _step)); }
  // The likeliest hsync that begins in bins [k0, k1], where there is one by its shape (pulse_at, edge_of) as deep
  // as kMinDepth. How far it has to stand out of the noise is the caller's to say.
  std::optional<FoldPulse> pulse(int k0, int k1);

private:
  static constexpr float kMinDepth = 0.02f;  // MHz
  float mean(int a, int b) const { return (_ps[_o + b] - _ps[_o + a]) / static_cast<float>(b - a); }
  float pulse_at(int k, float &tip, float &blank) const;
  bool edge_of(int e, float tip, float blank, double &edge) const;

  double _us = 1;       // samples in a microsecond
  int _step = 1;
  bool _wraps = false;
  size_t _o = 0;           // where bin 0 is in the sums
  std::vector<float> _ps;  // sums of v up to each bin; three times round when it wraps
  // where a pulse that begins at bin 0 has its front porch, its tip and its back porch (pulse_at), and the
  // stretches its edge is measured over (edge_of): in bins
  int _front[2] = {}, _tip[2] = {}, _back[2] = {};
  int _e_half = 1, _e_from = 0, _e_to = 0, _e_before = 0, _e_low = 0;
};

}  // namespace kp
