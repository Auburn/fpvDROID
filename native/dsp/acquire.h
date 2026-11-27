// fpvDROID decoder: acquisition. What a buffer of the sync path's samples (60 ms) says of the signal, two ways:
//  - By its pulses, the spike's way (score_pulses): with the levels from a histogram of the buffer
//    (SyncLevels::survey), the pulses of an hsync's length between two crossings of the threshold, and how many of
//    them come a line apart.
//  - fpvDROID: by the fold of its lines (acquire_by_fold), for a signal too weak for that. The first way wants two
//    peaks in a histogram for the levels and then pulses between crossings of a threshold: in noise as large as the
//    sync is deep it finds neither, at any picture. The fold of the lines does (fold.h). The 60 ms are folded at
//    the standard's line period (or the transmitter's, where it is known from before) in kAcqParts parts. In each
//    the hsync is looked for, and must be what lies lowest in the whole line. A transmitter's line period is a few
//    tens of parts in a million off the standard's, so the pulse moves along from part to part: at least five of
//    the eight must have it on one straight line, to 0.5 us, and that line's slope is the period. (Five places in
//    a 63 us line that agree so by chance: once in hours of pure noise, and then the fold does not show a pulse,
//    and the lock is given up again: LineHold::lost.)
// Neither changes anything: the decoder does what they say (video.cpp: acquire).
#pragma once
#include <cstddef>
#include <optional>
#include <vector>

#include "video_std.h"

namespace kp {

struct PulseScore {
  size_t pulses;  // pulses of an hsync's length found
  float score;    // the share of them that came a line after the one before
  double T_us;    // ... and the line period by those: the one in the middle (0: none did)
};
// buf[0..cnt) read with the polarity sgn against the threshold thr. Nothing: fewer than 30 such pulses.
std::optional<PulseScore> score_pulses(const std::vector<float> &buf, size_t cnt, double fs, float sgn, float thr);

constexpr int kAcqParts = 8;
struct FoldLock {
  const VideoStd *std;  // the standard whose lines fold
  int parts;            // how many of the kAcqParts parts have the hsync on one line
  double T;             // the line period, samples
  double edge;          // where the fold has the last hsync in the buffer, samples from its start
  float tip, blank;     // the levels, by the polarity
};
// buf[0..cnt) folded, with the polarity sign (the one in use: the one found last, the usual one before that).
// std_force: DecoderOpts::std_force. T_known: the line period of the standard std_now's transmitter, where it is
// known from before.
std::optional<FoldLock> acquire_by_fold(const std::vector<float> &buf, size_t cnt, double fs, float sign, int std_force,
                                        const VideoStd *std_now, std::optional<double> T_known);

}  // namespace kp
