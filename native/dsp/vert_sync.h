// fpvDROID decoder: the vertical sync of a weak signal. The decoder finds the broad pulses by how long the signal
// stays under the threshold in one run (video.cpp: close_line), and in noise as large as the sync is deep no run
// lasts. But the signal's mean over the middle of a half line says what that half line is, noise or not: at the
// tip's level in a broad pulse, at blanking's in an equalising one, at or above it in the picture. So each line's
// two half lines are measured (0: the tip's level, 1: blanking's), and a vertical sync is the standard's six half
// lines (PAL five) nearer the tip, with one nearer blanking either side. It is known three lines after the line
// it began in, still well before the field's first line of picture. What is done with it: FieldRhythm::seen.
#pragma once
#include <cstdint>
#include <optional>

#include "field_rhythm.h"
#include "rings.h"
#include "sync_levels.h"

namespace kp {

class VertSync {
public:
  // So many lines after a field was due is a vertical sync known this way: the flywheel waits them, with a weak
  // signal.
  static constexpr int kWait = 4;
  // The half lines so far are of another grid, or of another signal.
  void reset() { _n = 0; }
  // Line `line` has ended, which began at ls and was T long (samples; fs: the sample rate): its two half lines
  // are measured, against the levels as the decoder has them. The start of the field whose vertical sync has all
  // of its broad pulses, and the half line after them, in with this line; nothing when there is none.
  std::optional<FieldStart> line(const SyncSamples &s, double ls, double T, double fs, float sign, LevelPair levels,
                                 bool pal, int64_t line);
  // One of the last line's half lines was a broad pulse.
  bool broad() const { return _n >= 2 && (_q[(_n - 1) & 15] < 0.5f || _q[(_n - 2) & 15] < 0.5f); }

private:
  float _q[16] = {};  // the last half lines
  uint64_t _n = 0;    // half lines measured
};

}  // namespace kp
