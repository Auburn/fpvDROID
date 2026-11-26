// fpvDROID: digital video told apart by itself on the Spectrum tab (DJI, HDZero, other digital systems), from what a
// sweep step takes: its spectrum and its 12 ms of contiguous IQ (as a StepGrid). Nothing is decoded: it only says that
// a signal is digital, which system it looks like, where its centre is and how wide it is.
//
// Ported from FPVDesktopWiFiReceiver (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver, GPL-2.0: src/kp_ext/digital_scan.cpp
// and test/digital_scan_test.cpp, from its pull requests 2 and 15), where it runs in the scan. What it looks for:
// - A wide, flat block in the spectrum, 12 to 31 MHz (OFDM: every subcarrier at the same power, sides that drop
//   almost straight down). Analog FM video is not flat: its spectrum is the picture's histogram of levels, with
//   peaks at sync and at black.
// - How the block comes and goes in time. HDZero sends without a break; DJI sends in a TDD frame of 2.5 ms (the
//   goggles' uplink goes in the gaps: measured upstream on a real DJI transmitter on R2); Wi-Fi comes in bursts with
//   no rhythm. Both measures need contiguous samples, which a sweep step's capture is.
// - Then by width and frequency (judge): a 2.5 ms frame is DJI; continuous and 22 to 31 MHz wide on an HDZero channel
//   is HDZero, 14 to 22 MHz there its narrow mode (540p60); anything else is another digital system.
// A step sees +-16.5 MHz, so a signal often does not fit in one: one cut by a step's upper edge and by the next
// step's lower edge is joined into one.
//
// Upstream's DJI figures come from recordings of a real DJI transmitter; its HDZero figures from synthetic signals
// only (OFDM of 27 or 17 MHz, as Divimath describes it), so the HDZero thresholds may need tuning with real captures.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "spectrum.h"
#include "step_grid.h"

namespace kp {

// A wide flat block in one step's spectrum: its band relative to the step's centre, as find_wide_regions gives it, and
// on the band (absolute) once DigitalScan::step has put it there.
struct WideRegion {
  double f0_hz = 0, f1_hz = 0;            // the band it takes, at the floor + 6 dB
  bool clip_lo = false, clip_hi = false;  // it reaches the edge of the step's span: the signal goes on past it
  float snr_db = 0;                       // its median level over the noise floor
  float flat = 0;                         // the fraction of it within 3 dB of that median
  float duty = 0;                         // the fraction of ~0.1 ms slices it is there in (band_duty)
  double frame_s = 0;                     // its TDD frame (tdd_frame); 0: no regular gaps
};

// The wide flat blocks of a step: sm its spectrum through smooth_db (Psd::db(): fftshifted, fs wide), edge_hz the span
// used (+-).
std::vector<WideRegion> find_wide_regions(const std::vector<float> &sm, double fs, double edge_hz);

// How much of the time the band f0..f1 (relative to the centre) is on the air in the step's grid: 0..1.
float band_duty(const StepGrid &g, double f0_hz, double f1_hz);

// A TDD frame in the band f0..f1: a send/quiet pattern that repeats with a fixed period of 1 to 4 ms. The period in
// seconds, 0 if there is none. DJI's, measured upstream (20 MHz on R2): 2.5 ms; with video, ~1.73 ms sending, a gap
// of ~0.25 ms, ~0.37 ms sending and a gap of ~0.1 ms; with no goggles linked, 0.41 ms, a gap of 0.08, 0.43 ms and a
// gap of 1.53 ms.
double tdd_frame(const StepGrid &g, double f0_hz, double f1_hz);

enum class DigitalKind { None, HDZero, HDZeroNarrow, DJI, Other };
// Every rule in one place: what a block whose band is on the band (absolute Hz) looks like, by its width, its level,
// how flat it is, its timing and where it is. None: not judged digital (too narrow or too wide to be judged, among
// others: DJI's 40 and 60 MHz modes are wider than a step, and are never joined into one).
DigitalKind judge(const WideRegion &block);
// The signal's "kind" in spectrum_json: "hdzero", "hdzero_narrow", "dji" or "digital"; "" for None.
const char *digital_code(DigitalKind k);
// ... and its "label", the system as the Spectrum tab names it: "HDZero", "HDZero narrow", "DJI" or "Digital".
const char *digital_label(DigitalKind k);

// A digital signal found, in absolute frequencies.
struct DigitalSignal {
  double centre_hz = 0, width_hz = 0;
  float snr_db = 0;
  DigitalKind kind = DigitalKind::None;
  int64_t seen_ms = 0;  // when it was last found (DigitalMap)
};

// The steps of a sweep's pass, one after another in the order they are taken (upwards): each step's wide flat blocks
// timed and judged, and a block cut by this step's lower edge joined with the other half the step just below saw.
// Only the blocks that may be digital are timed. The DSP thread's alone.
class DigitalScan {
public:
  // A new sweep: nothing to join with.
  void reset() { _prev.clear(); }
  // One step: its spectrum through smooth_db, its grid (StepGrid::build), the hardware centre on centre_hz, +-edge_hz
  // of it used. The digital signals whole in it, or joined with the step below.
  std::vector<DigitalSignal> step(const std::vector<float> &sm, const StepGrid &grid, double centre_hz, double edge_hz);

private:
  std::vector<WideRegion> _prev;  // the step before's blocks, on the band
  double _prev_centre = 0;
};

// The verdicts of the sweep, kept per frequency from pass to pass the way BandMap keeps levels: one step may see a
// signal cut and the two grids of a sweep see different halves of it, so a signal is judged in some steps and not
// in others. A verdict stays until the steps over it have not found it again for kKeepMs. Not thread-safe.
class DigitalMap {
public:
  // Two passes on each grid of the 5.8 GHz band (~2 s a pass): a signal judged on one grid only is found every
  // other pass, and can be missed once without the verdict going away.
  static constexpr int64_t kKeepMs = 8000;
  // Within this of each other, two verdicts are of one signal (a new one takes the old one's place).
  static constexpr double kSameHz = 4e6;

  void add(const std::vector<DigitalSignal> &found, int64_t now_ms);
  // Verdicts centred in lo..hi last found before older_than_ms are forgotten.
  void forget(double lo_hz, double hi_hz, int64_t older_than_ms);
  const std::vector<DigitalSignal> &all() const { return _kept; }

private:
  std::vector<DigitalSignal> _kept;
};

// The verdict for a carrier of the band picture (find_carriers), or nullptr: the nearest within DigitalMap::kSameHz
// of its centre, of a width that could be the same signal's.
const DigitalSignal *digital_for(const std::vector<DigitalSignal> &verdicts, const Carrier &c);

}  // namespace kp
