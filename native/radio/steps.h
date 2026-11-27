// fpvDROID radio: the retunes of a scan or a sweep, one after another, and the capture of the one under way.
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "video.h"

namespace kp {

// A plan of retunes (a scan's, a pass of a sweep) and the capture of the step it is on. A step is given the banks
// of its own retune only (the caller sorts those out). The first kSettleBanks are let go by (the chip's AGC), then
// need() samples are taken in a row. A gap (banks dropped on the way, or the stream restarted) starts the capture
// over. Not for ever: on a link that loses a bank every few ms no capture would ever be whole and the plan would stay
// on this step, so after kRestarts tries it is taken with its gaps (the sync check loses a few lines at each).
//
// The DSP thread's alone. The capture is kept as int16 pairs: a scan step's 90 ms at 40 MS/s is 3.6 M samples.
class Steps {
public:
  static constexpr int kSettleBanks = 40;  // ~8 ms after a retune
  static constexpr int kRestarts = 8;

  // A plan from its first step, each step's capture `need` samples to begin with (plan: at least one step).
  void start(std::vector<double> plan, size_t need);
  // On to the next step. False: that was the last. Either way the capture stays the last step's (iq(), centre_hz())
  // until the next step's first bank.
  bool next();
  bool done() const { return _at >= _plan.size(); }
  size_t at() const { return _at; }
  double hz() const { return _plan[_at]; }  // the step to retune to (not done())
  // Steps to take right after the one under way (not done()), before the rest of the plan, in this order: a scan's
  // extra steps (scan.h). size() counts them from here on.
  void insert(const std::vector<double> &hz);
  size_t size() const { return _plan.size(); }

  // One bank of the step's retune (path A; the hardware centre on centre_hz): true once need() samples are in.
  bool take(const std::vector<cf> &bank, uint64_t seq, uint32_t gen, double centre_hz);
  size_t need() const { return _need; }
  // This step's capture is to go on to n samples (the scan's sync check, after the spectrum of the first few).
  void want(size_t n) { _need = n; }
  std::vector<cf> iq() const;
  double centre_hz() const { return _centre; }

private:
  std::vector<double> _plan;
  size_t _at = 0;
  size_t _first_need = 0, _need = 0;
  int _settle = -1;  // banks still to let go by; < 0: the step's first bank is still to come
  int _restarts = 0;
  uint64_t _last_seq = 0;
  uint32_t _gen = 0;
  double _centre = 0;
  std::vector<std::complex<int16_t>> _cap;
};

}  // namespace kp
