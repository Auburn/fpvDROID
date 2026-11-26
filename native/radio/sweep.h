// fpvDROID radio: the Spectrum tab's sweep (the steps it retunes to, pass after pass) and the band picture as the app
// reads it (band_json).
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "digital.h"

namespace kp {

// A sweep over lo..hi, pass after pass. A pass retunes every kStep, the centres on the 5 MHz grid where the chip
// puts its hardware centre, so every step is centred where it was asked to be. Passes alternate between two grids
// half a step apart: what one pass has at a step's DC point (left out of the picture) the next has 15 MHz off
// centre.
//
// Not thread-safe: the Radio keeps it under its lock (start() from the caller's thread, the passes from the DSP
// thread, passes() from the spectrum poll).
class Sweep {
public:
  // Steps this far apart: each one's +-16.5 MHz (radio.cpp's kEdge) overlaps its neighbours' by 3 MHz.
  static constexpr double kStep = 30e6;
  // The band picture (BandMap) the Radio keeps: every analog FPV channel (L1 5362 .. E8 5945) with room either side,
  // in half-MHz bins. A sweep's span is kept inside it.
  static constexpr double kBandLo = 5300e6, kBandHi = 6000e6, kBandBin = 0.5e6;
  // What a new sweep finds older than this in its span (a scan an hour back) is forgotten.
  static constexpr int64_t kStaleMs = 60000;

  // One pass over lo..hi on grid 0 or 1: at least one step.
  static std::vector<double> plan(double lo_hz, double hi_hz, int grid);

  // A new sweep over lo..hi (either order, kept inside the band picture): no pass finished, the first on grid 0.
  void start(double lo_hz, double hi_hz);
  double lo() const { return _lo; }
  double hi() const { return _hi; }
  // The steps of the pass to take now.
  std::vector<double> pass() const { return plan(_lo, _hi, _grid); }
  // The pass under way is finished: the next is on the other grid.
  void pass_done() {
    _grid ^= 1;
    _passes++;
  }
  uint64_t passes() const { return _passes; }

private:
  double _lo = kBandLo, _hi = kBandHi;
  int _grid = 0;
  uint64_t _passes = 0;
};

// The band picture over a span, as Radio::spectrum_json gives it (the format is there): db from BandMap::get, its
// first bin at first_hz, bin_hz apart; at_hz the step being taken now (0: none), at_width_hz what a step covers;
// sweeps: passes finished. The carriers in it are found here (find_carriers), each with the kind of digital signal
// the sweep judged it to be, if any (digital: DigitalMap::all(), matched by digital_for).
std::string band_json(const std::vector<float> &db, double first_hz, double bin_hz, double at_hz, double at_width_hz,
                      uint64_t sweeps, const std::vector<DigitalSignal> &digital = {});

}  // namespace kp
