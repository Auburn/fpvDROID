// fpvDROID radio: the scan's plan, and its extra steps for signals that no step of the plan sees whole.
#pragma once
#include <cstddef>
#include <utility>
#include <vector>

#include "spectrum.h"

namespace kp {

// The scan's retunes (Hz): the common analog bands first (A, B, E, F, R: 5645..5945), then the low band (L:
// 5362..5621), 20 MHz apart. The chip puts each on the 5 MHz grid (5652 -> 5650).
std::vector<double> scan_plan();

// A scan only takes the humps a step sees whole (find_humps), and with steps 20 MHz apart, each seeing +-16.5 MHz,
// a signal is sure to be whole in one only up to 13 MHz wide. A VTX close by is 15-20 MHz wide: one halfway between
// two steps (F2 at 5760 MHz, between the 5750 and 5770 steps) is cut by an edge of both and the scan never lists
// it, and nor does it list E8 (5945), cut by the top edge of the highest step. Such a signal gets a step of its own.
// The idea, a step centred on a signal that a step's edge cuts, at most 8 a scan and none for one seen whole, is
// FPVDesktopWiFiReceiver's (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver, GPL-2.0: Radio::add_cut_steps).
//
// Unlike there, the extra step is not asked for at once: a step's cut signals wait for the next step of the plan.
// It may see one whole after all (a signal nearer to that step: no extra step), or its other end, cut by its other
// edge (then both ends are known, and the extra step is centred between them; a signal more than 24 MHz wide gets
// none, as find_humps would not take it). Only then is the extra step asked for, to be taken next. One cut at the
// plan's last step, or not seen at all by the next (the plan goes on in another band), gets its step at once, from
// the one end seen. A signal seen whole in any step, or inside a step already given, gets none; at most kMax a scan,
// so that a band full of Wi-Fi does not make a scan much longer. What an extra step itself sees cut asks for nothing
// more.
//
// The DSP thread's alone; plain values in and out, so that it is tested without an adapter.
class CutSteps {
public:
  // An extra step is centred so that this much either side of the middle of the signal fits (find_humps takes humps
  // up to 24 MHz wide), or, for a signal of which only one end was seen, this far inside that end. The chip puts the
  // hardware centre on the 5 MHz grid, up to 2.5 MHz from the step asked for: 12 + 2.5 is inside the 16.5 a step
  // sees.
  static constexpr double kReach = 12e6;
  // A signal's step is asked for only if one of the parts seen of it stands this far over the floor (weaker: a wide
  // Wi-Fi's skirts, noise). A weaker part still counts as one end of a stronger one.
  static constexpr float kMinSnrDb = 10;
  static constexpr size_t kMax = 8;

  struct Extra {
    double hz = 0;                // the step to take
    double lo_hz = 0, hi_hz = 0;  // the signal's ends, as far as they were seen (0: that end was not)
    float snr_db = 0;             // the stronger of the parts seen
  };

  void start();
  // One step's spectrum, with the hardware centre centre_hz: the humps find_humps took from it (whole) and what
  // find_cut_signals found cut, both relative to centre_hz. edge_hz: what a step sees either side (find_humps').
  // extra: this step is one of those asked for here (its cut signals are not looked at); last: the plan has no step
  // after this one. Returns the steps to take next, in order.
  std::vector<Extra> step(double centre_hz, double edge_hz, const std::vector<Hump> &whole,
                          const std::vector<CutSignal> &cut, bool extra, bool last);
  size_t given() const { return _given.size(); }

private:
  struct Part {
    double f0, f1;  // Hz, the part a step saw
    bool cut_hi;    // it goes on past that step's top edge (f0 is the signal's bottom end); else past its bottom one
    float snr_db;
  };
  bool covered(const Part &p, double edge_hz) const;
  static Extra one_end(const Part &p);
  void give(const Extra &e, std::vector<Extra> &out);

  std::vector<std::pair<double, double>> _whole;  // Hz: signals seen whole in this scan
  std::vector<double> _given;                     // the extra steps asked for in this scan
  std::vector<Part> _pending;                     // cut in the last step of the plan, waiting for the next
};

}  // namespace kp
