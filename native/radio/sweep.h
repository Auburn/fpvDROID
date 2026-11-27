// fpvDROID radio: the Spectrum tab's sweep. The steps it retunes to, pass after pass (Sweep); what each step's capture
// is measured for, on the DSP thread (StepAnalysers); and the band picture those measures go into, as the app reads it
// (BandPicture, under the Radio's lock).
//
// Everything the sweep recognises on the air plugs in at four places: a field of StepFindings (what one step says), its
// measure in StepAnalysers::step (from the step's spectrum or its StepGrid), what is kept of it in BandPicture (added
// in add, forgotten in sweep_started), and its spans in BandPicture::json's "signals" (with the carriers on them
// tagged with its kind). Radio::handle_sweep and the tests take every step the same way: StepAnalysers::step, then
// BandPicture::add.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "digital.h"
#include "spectrum.h"
#include "step_grid.h"

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
  // Steps this far apart: each one's +-16.5 MHz (ScanPlan::kEdge) overlaps its neighbours' by 3 MHz.
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

// What one sweep step's capture says (StepAnalysers::step), for the band picture to take in (BandPicture::add).
struct StepFindings {
  double centre_hz = 0;  // the hardware centre
  double fs = 0;         // the sample rate
  double edge_hz = 0;    // +- of the centre used
  std::vector<float> spectrum;         // the step's spectrum (Psd::db()) through smooth_db: the band picture's levels
  std::vector<DigitalSignal> digital;  // digital video whole in it, or joined with the step below (DigitalScan)
  // Wi-Fi plugs in here: the bursts of its 0.1 ms slices (wifi.h's BurstStep, measured off StepAnalysers' grid).
};

// The measures of a sweep's steps: the costly part, which needs a step's IQ, taken on the DSP thread with no lock held.
// A step is cut up once for every measure: its spectrum (a 1024-point Psd), smoothed once, and its StepGrid. The DSP
// thread's alone.
class StepAnalysers {
public:
  // A new sweep: no step before to join a signal with.
  void sweep_started() { _digital.reset(); }
  // One step: its capture (fs, in a row), the hardware centre on centre_hz, +-edge_hz of it used.
  StepFindings step(const std::vector<cf> &iq, double fs, double centre_hz, double edge_hz);

private:
  Psd _psd{1024};
  StepGrid _grid;
  DigitalScan _digital;
};

// Something the sweep has recognised on the air: a span of the band and what it is (spectrum_json's "signals").
struct Signal {
  double lo_hz = 0, hi_hz = 0;
  const char *kind = "";  // digital_code() for digital video; Wi-Fi adds "wifi"
  std::string label;      // what the Spectrum tab calls it (no quotes or backslashes in it)
  bool active = true;     // in use; false: on the air but idle
};

// The band picture the Spectrum tab draws, from the steps of sweeps and of scans: the levels (BandMap, over
// Sweep::kBandLo..kBandHi) and what the sweeps have recognised in them: digital video (DigitalMap). Not thread-safe:
// the Radio keeps it under its lock, so that a step's levels and what it recognised go in together.
class BandPicture {
public:
  BandPicture();
  // A sweep over lo..hi starts: what is older than Sweep::kStaleMs there is forgotten (a scan an hour back).
  void sweep_started(double lo_hz, double hi_hz, int64_t now_ms);
  // A sweep step: its levels, and what it recognised. A digital verdict within the step's reach that the steps over it
  // have not found again for DigitalMap::kKeepMs goes: the levels fade by themselves, being averaged with new ones,
  // but a verdict is only ever confirmed. Once the sweep stops, both stay as last seen.
  void add(const StepFindings &f, int64_t now_ms);
  // A scan step: its spectrum through smooth_db, into the levels alone (a scan judges nothing).
  void add_levels(const std::vector<float> &sm, double fs, double centre_hz, double edge_hz, int64_t now_ms);
  // The picture over lo..hi, as Radio::spectrum_json gives it (radio.h has the format): at_hz the step being taken now
  // (0: none), at_width_hz what a step covers, sweeps the passes finished. Its carriers are found here (find_carriers),
  // each tagged with the kind of the signal it is on, if any.
  std::string json(double lo_hz, double hi_hz, double at_hz, double at_width_hz, uint64_t sweeps) const;

  const BandMap &band() const { return _band; }
  const DigitalMap &digital() const { return _digital; }

private:
  BandMap _band;
  DigitalMap _digital;
  // Wi-Fi plugs in here: its WifiWatch, and lo..hi from sweep_started for the carriers it weighs bursts against.
};

}  // namespace kp
