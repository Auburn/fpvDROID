// fpvDROID decoder: the sync levels. Where the signal's sync tip and its blanking level are, in MHz of discriminator
// output, and so the threshold its sync pulses are found by. They are had three ways:
//  - From a histogram of the samples, the spike's way: at acquisition (survey), and after it until the next way
//    has settled (track).
//  - fpvDROID: sync-gated (measure): sync tip from the middle of each hsync pulse, blanking from the back porch,
//    measured on locked active lines and smoothed over ~64 lines. Picture content cannot move them (the spike's
//    histogram peaks could, and the threshold wandered with the scene).
//    ... or over 4, while they slide. The same AC coupling that makes the levels jump with a flashing light (next)
//    makes them slide with the picture down the field: one camera under a bright sky over dark ground moved them
//    0.85 sync depths down and up again in every field, a hundredth of the depth a line. Followed a 64th of the
//    way a line, the kept levels were a quarter of the depth behind before the jump below caught up: the lower
//    part of every picture a tenth of the full brightness too bright and then, from one line on, not (and that
//    line another one in every field: woven, alternate lines dark and bright). So: when the lines before were off
//    the kept levels one way by more than they scatter, the levels are sliding and this line is followed a
//    quarter of the way. Noise does not look like that, and a click in one line's sync is followed at the pace
//    the lines before it set.
//  - fpvDROID: ... and after a jump (look). A transmitter whose video input is AC-coupled moves the whole signal,
//    sync and picture alike, when the picture's brightness changes: a camera pointed at a light flashing at 7 Hz
//    moved it by 1.3 times the sync depth with every flash. The threshold is then outside the pulses, no line finds
//    its hsync, and the gated levels, measured on lines that did, never hear of it: the lock went with every flash.
//    So every line is looked at where its hsync is due, found or not (the grid freewheels on: it is the levels
//    that moved, not the timing), and when kJumpLines of them in a row show a pulse of the same depth as before at
//    the same new levels, the levels go there at once. That transmitter's signal took some milliseconds over each
//    move: so for kFastLines lines after a jump the gated levels follow a quarter of the way a line, not a 64th,
//    and the rest of the slide is followed line by line (else in steps, each a jump of its own, and the picture's
//    brightness with them).
#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "rings.h"

namespace kp {

struct LevelPair {
  float tip, blank;  // sync tip and blanking, MHz of discriminator output (by the polarity)
};

// What one line's sync says of the levels (SyncLevels::look).
struct LineLevels {
  std::optional<LevelPair> own;  // it is at other levels than the kept ones: these, which the line is drawn against
  bool jumped = false;           // ... and with this line the kept levels have gone there
};

class SyncLevels {
public:
  // follow: the kept levels go after a signal that jumps (look) and follow one that slides fast (measure). Without
  // (DecoderOpts::follow_levels off) they are the sync-gated ones at their own pace, and nothing else.
  SyncLevels(double fs, bool follow);

  float sign = +1;                         // the polarity: +1 with the sync at the low-frequency end
  float tip = -1, blank = 0, thr = -0.5f;  // the kept levels, and the threshold half way between them
  uint64_t jumps = 0;                      // times they have gone to a signal that jumped

  // The mean of R.luma over [from, to), by the polarity, into out; false when that stretch is not in the ring.
  // (Not a NaN return: this is built with -ffast-math, under which a test for NaN may be compiled away.)
  bool mean(const SampleRings &R, double from, double to, float &out) const;

  // Acquisition: the levels in buf[0..cnt) read with the polarity sgn; false when it shows none.
  bool survey(const std::vector<float> &buf, size_t cnt, float sgn, float &tip_o, float &blank_o);
  // ... and what acquisition settled on.
  void acquired(float sgn, float tip_a, float blank_a);
  // The levels as measured another way (in the fold of many lines, with which the decoder holds a weak signal:
  // line_hold.h, acquire.h). They are the kept ones now, and the sync-gated measurement counts as settled: no more
  // of the histogram's.
  void set(float tip_m, float blank_m);
  // The lock is lost: the sync-gated measurement starts over, and a jump half seen is forgotten.
  void restart();

  // The sync-gated measurement has settled. Until it has, every sample of the sync path goes to track().
  bool settled() const { return _gated >= kSettle; }
  void track(float m);
  // A locked picture line began at ls, with its sync at the kept levels: they follow it, fast while they slide.
  // (The vertical interval's lines with an hsync were measured too for a while, for the slide to go on through
  // it: the lit recordings came out no different without, and a weak recording lost half a percent of its lines.)
  void measure(const SampleRings &R, double ls);
  // Does the line that began at ls (its hsync found there, or only due there) have its sync at other levels than
  // the kept ones? It does when there is an hsync at ls by its shape (a pulse flat at its bottom, between two
  // porches at one level, as deep as the sync was) and both its levels are off the kept ones the same way by more
  // than a quarter of that depth. (A click moves one of the two, noise neither that far; where the grid is off
  // the signal there is no such pulse, and a broad pulse has no porch behind it: one PAL transmitter's dip 2 us
  // into its broad pulses, as deep again as the sync, would else pass for an hsync one depth down.) The
  // kJumpLines-th such line in a row at the same new levels moves the kept levels onto them. A line with an hsync
  // where the levels were ends the run at once, lines that cannot say (the vertical interval's) after a few.
  // (Without `follow`, no line says anything.)
  LineLevels look(const SampleRings &R, double ls);

  // Where a line's sync is read, in us from its hsync's leading edge: the tip inside the pulse (4.7 us long) and
  // the back porch behind it (the burst averages out), each with its middle for look()'s two halves, and the
  // front porch ahead of the edge.
  // (The porch only as far as 8.2 us: one camera's picture begins 8.7 us after the edge, not the standard's 9.4,
  // and a bright left edge then read as a porch that was higher behind.)
  static constexpr double kTipUs[3] = {1.2, 2.4, 3.6}, kPorchUs[3] = {5.8, 7.0, 8.2}, kFrontUs[2] = {1.1, 0.4};

private:
  static constexpr uint64_t kSettle = 64;  // gated lines
  static constexpr int kJumpLines = 3, kFastLines = 128;
  // A slide (sliding): how far the lines are off the kept levels is averaged over so many lines (_drift), how much
  // they scatter about that over so many (_scatter), and the first has to be so many times what the second makes
  // of such an average.
  static constexpr int kDriftLines = 8, kScatterLines = 32;
  static constexpr float kSlideSigmas = 3;

  bool from_hist(uint32_t total, float &tip_o, float &blank_o);
  bool sliding() const;

  double _fs;
  bool _follow;
  std::vector<uint32_t> _hist;  // of the samples, by the polarity, over [_hlo, _hhi) MHz
  double _hlo = -8, _hhi = 8;
  int _hdec = 0;
  uint64_t _hist_count = 0;
  uint64_t _gated = 0;                // lines measured sync-gated since acquisition
  int _fast = 0;                      // lines the gated levels still follow fast for
  // How far the lines lately were off the kept levels (tip and blanking together) on average, and how much they
  // scatter about that (mean square): a slide is the first being more than the second makes of it.
  float _drift = 0, _scatter = 0;
  int _run = 0, _gap = 0;             // lines that have said the levels moved, and lines since that could not say
  float _run_tip = 0, _run_blank = 0;  // the levels they said (their mean)
};

}  // namespace kp
