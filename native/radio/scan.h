// fpvDROID radio: the scan's plan, and the extra steps it takes for a signal that no step of the plan sees whole.
#pragma once
#include <cstddef>
#include <utility>
#include <vector>

#include "spectrum.h"

namespace kp {

// The scan's retunes, kStep apart: the common analog bands first (A, B, E, F, R: 5645..5945 MHz), then the low band
// (L: 5362..5621). The chip puts each hardware centre on its kGrid (5652 -> 5650), and a step sees +-kEdge of it.
//
// A step takes the humps it sees whole (find_step_signals), so a signal is sure to be whole in one only up to
// 2 kEdge - kStep = 13 MHz wide. A VTX close by is 15-20 MHz wide: one halfway between two steps (F2 at 5760 MHz,
// between the 5750 and 5770 steps) is cut by an edge of both and the scan never lists it, and nor does it list E8
// (5945), cut by the top edge of the highest step. Such a signal gets a step of its own. The idea, a step centred on a
// signal that a step's edge cuts, at most 8 a scan and none for one seen whole, is FPVDesktopWiFiReceiver's
// (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver, GPL-2.0: Radio::add_cut_steps).
//
// Unlike there, what a step sees cut first waits for the next step of the plan. It may see the signal whole after all
// (no extra step), or its other end, cut by its other edge (then both ends are known, and the extra step is centred
// between them; a signal wider than kHumpMaxHz gets none, as no step would take it whole). One cut at the plan's last
// step gets its step at once, from the one end seen; one that the next step does not see at all (the plan goes on in
// another band) gets it then. A signal seen whole in any step, or inside an extra step already given, gets none; at
// most kMax a scan, so that a band full of Wi-Fi does not make a scan much longer. The extra steps are taken right
// after the step that gave them, and what they see cut asks for nothing more.
//
// The DSP thread's alone; plain values in and out, so that it is tested without an adapter.
class ScanPlan {
public:
  static constexpr double kStep = 20e6;
  static constexpr double kEdge = 16.5e6;  // usable +- span at 40 MS/s (BW40 DFIR is flat to ~17 MHz)
  static constexpr double kGrid = 5e6;     // the chip puts each hardware centre on this grid
  // An extra step is centred so that this much either side of the middle of the signal fits (the widest hump
  // find_step_signals takes), or, for a signal of which only one end was seen, this far inside that end. The chip
  // moves the step up to half a kGrid, and it still sees that far.
  static constexpr double kReach = kHumpMaxHz / 2;
  static_assert(kReach + kGrid / 2 <= kEdge, "an extra step on the chip's grid sees kReach either side of its signal");
  // A signal's step is asked for only if one of the parts seen of it stands this far over the floor (weaker: a wide
  // Wi-Fi's skirts, noise). A weaker part still counts as one end of a stronger one.
  static constexpr float kMinSnrDb = 10;
  static constexpr size_t kMax = 8;

  struct Extra {
    double hz = 0;                // the step to take
    double lo_hz = 0, hi_hz = 0;  // the signal's ends, as far as they were seen (0: that end was not)
    float snr_db = 0;             // the stronger of the parts seen
  };

  // The plan (Hz), in the order its steps are taken.
  static std::vector<double> plan();
  // A new scan: its plan, and nothing of the last scan remembered.
  std::vector<double> start();
  // Every step of the scan as it is taken, the plan's and the extra ones in their order: its hardware centre, and what
  // find_step_signals found in its spectrum (relative to centre_hz). Returns the extra steps to take next, in order.
  std::vector<Extra> step(double centre_hz, const std::vector<Hump> &whole, const std::vector<CutSignal> &cut);
  size_t given() const { return _given.size(); }
  // The steps to take, as Steps::insert takes them.
  static std::vector<double> hz_of(const std::vector<Extra> &extra);

private:
  struct Part {
    double f0, f1;  // Hz, the part a step saw
    bool cut_hi;    // it goes on past that step's top edge (f0 is the signal's bottom end); else past its bottom one
    float snr_db;
  };
  bool covered(const Part &p) const;
  static Extra one_end(const Part &p);
  void give(const Extra &e, std::vector<Extra> &out);

  std::vector<double> _plan;
  size_t _at = 0;    // the plan's step that step() is called for next, once the extra steps owed are taken
  size_t _owed = 0;  // extra steps given and not yet taken: the next _owed calls of step() are those steps
  std::vector<std::pair<double, double>> _whole;  // Hz: signals seen whole in this scan
  std::vector<double> _given;                     // the extra steps asked for in this scan
  std::vector<Part> _pending;                     // cut in the plan's step before, waiting for its next
};

}  // namespace kp
