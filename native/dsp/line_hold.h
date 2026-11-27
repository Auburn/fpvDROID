// fpvDROID decoder: holding the line grid through a weak signal (DecoderOpts::hold). An FM signal that sinks into
// the noise does two things to the sync pulses. Each line's own hsync is no longer found: a pulse of the right
// length between two crossings of a threshold is a matter of chance once the noise is as large as the pulse is
// deep. And the levels move: the discriminator is captured by the noise, whose mean is the middle of the band, and
// tip and blanking both shrink towards it, until the threshold kept from better times is outside the pulses. The
// lock was then lost and looked for again, 60 ms without a picture at a time, with the levels and the fields'
// rhythm forgotten: four times a second in recordings of such a signal (the noise in the sync path as large as the
// sync was deep). All of that for nothing: the camera's lines come from a crystal, and in those recordings they
// stay where a line period says, to a fraction of a microsecond, for as long as each lasts.
// So the lines are folded (fold.h): the sync path's samples around where each line's hsync is due, a leaky mean
// over kFoldLines lines, in which the pulse stands out of the noise eight times better than in any one line.
//  - When fewer than kWeak of the last 64 lines find their own hsync the fold takes over (holding). It says where
//    the pulse is, and the grid is steered onto it; and at which levels, and tip and blanking follow them. A
//    line's own hsync then only counts as found, and only within kTightUs of the grid (pulses). kStrong of 64
//    lines finding theirs again ends it.
//  - While the fold does not show the pulse either, the grid runs on at the line period. That is known from a
//    long stretch of lines (period) far better than the lines' own tracking has it: to a few parts in ten
//    million, a fraction of a microsecond over a second. The lock is lost only when that has lasted kHoldS (lost).
//  - With some noise in the signal but each line's hsync still steering, the fold guards the grid: a pulse far
//    from where the fold shows the hsync is not taken for one (pulses).
//  - ... and when the signal has moved. The fold looks kLookUs either side of the grid. A block of samples lost on
//    the way here (the phone drops one a few times a minute) puts the hsyncs 14 us from it: with a good signal six
//    of them in a row re-phase the grid (the decoder's own way), with a weak one no six lines find theirs, the
//    fold shows nothing, and the lock would be given up after kHoldS and looked for again. So while the fold holds
//    the grid, the whole line is folded as well, coarsely (every fourth sample will do at the sync path's 1 MHz),
//    and when the fold has shown no hsync for a few milliseconds that one is looked at: an hsync by its shape, the
//    lowest thing in the line, well out of the noise, away from the grid, and in the same place at two looks 32
//    lines or more apart, moves the grid there (Verdict::rephased).
// A weak signal's fields are found another way too (vert_sync.h), and one that is weak from the start is acquired
// by the fold of its lines (acquire.h).
//
// Samples of the sync path and line starts come in here; where the next line begins, the line period and the
// levels go out, and the decoder does what they say.
#pragma once
#include <bit>
#include <cstdint>
#include <optional>

#include "fold.h"
#include "rings.h"
#include "sync_levels.h"

namespace kp {

class LineHold {
public:
  // A pulse this near to where the hsync is due counts as the line's, held; and is taken for it, guarded.
  static constexpr double kTightUs = 1.0, kNearUs = 1.5;

  explicit LineHold(double fs);
  // A new standard, with its line period T (samples): nothing is known of the transmitter's own any more.
  void set_std(double T);
  // The lock is lost.
  void restart();
  // The signal was acquired by the fold of its lines: held from its first line on.
  void acquired();
  // The decoder has moved the grid onto a run of hsyncs somewhere else. Six hsyncs running: the signal is no weak
  // one. What the fold holds was folded on the old grid.
  void jumped();

  // A line has ended, its own hsync found or not.
  void count(bool found) { _found = (_found << 1) | (found ? 1u : 0u); }

  struct Line {
    double start, next;  // where the line that has ended began, and where the one after it begins
    int64_t idx;         // which line it is, as the decoder counts them ...
    long long in_field;  // ... and of the field in hand (for the log)
    uint64_t lines;      // lines since the acquisition
    bool broad;          // a line of broad pulses: it has no porch behind its pulse, and stays out of the fold
    double T;            // the line period, samples
    float sign;          // the polarity, and the levels as the decoder has them
    LevelPair levels;
  };
  struct Verdict {
    double next, T;                   // where the next line begins, and the line period: as they were, or steered
    std::optional<LevelPair> levels;  // the levels, where the fold has a say in them
    bool rephased = false;            // `next` is on a signal that had moved (a jump in the stream)
  };
  // The line goes into the fold, and with a weak signal the fold steers.
  Verdict line(const SyncSamples &s, const Line &l);

  // What a pulse of an hsync's length is to the grid now.
  //  steer:   the line's hsync, anywhere in the line PLL's window: the grid follows it;
  //  guarded: the same, but one further than kNearUs from the grid is no hsync (some lines miss theirs, and the
  //           fold shows the hsync where the grid is);
  //  counted: held: within kTightUs of the grid it counts as the line's, and steers nothing.
  enum class Pulses { steer, guarded, counted };
  Pulses pulses() const { return _weak ? Pulses::counted : _fold_good && !clean() ? Pulses::guarded : Pulses::steer; }

  bool holding() const { return _weak; }
  uint64_t holds() const { return _holds; }  // how often that has begun
  // Holding, or within the last `lines` lines.
  bool lately(int lines) const { return _weak || _weak_ago < lines; }
  // Nearly every line finds its own hsync.
  bool clean() const { return !_weak && found64() >= kClean; }
  // Holding, and the fold has not shown the hsync for kHoldS (silent_s: for how long): the lock is lost.
  bool lost(double T) const { return _unseen * T > kHoldS * _fs; }
  double silent_s(double T) const { return _unseen * T / _fs; }
  // The transmitter's line period, where a long stretch of lines has shown it (samples).
  std::optional<double> period() const { return _long_ok ? std::optional<double>(_T_long) : std::nullopt; }
  void forget_period() { _long_ok = false; }
  // How far behind an hsync as the pulses find it the fold has its edge (samples).
  double ref() const { return _ref; }

private:
  static constexpr int kFoldLines = 64, kFoldMin = 16;
  static constexpr double kFoldBeforeUs = 7, kFoldUs = 21;  // the fold: from so long before each line's start, so long
  static constexpr double kLookUs = 4.5;   // ... and an hsync is looked for this far either side of the grid
  static constexpr double kHoldS = 0.5;
  static constexpr double kSteer = 1.0 / 16;  // of the way to where the fold has the pulse, every fourth line
  // Of the last 64 lines, how many find their own hsync: fewer than kWeak, and the fold takes over; kStrong, and
  // it lets go; kSure, and the grid is one to come back to; kClean, and the signal is a clean one.
  static constexpr int kWeak = 32, kSure = 52, kClean = 58, kStrong = 60;
  // Where the fold has the hsync's edge, from the line's start, while each line's own hsync steers (_ref): a
  // little later, because an hsync is found where the sync path's signal crosses the threshold, the fold measures
  // the edge by area, and behind that path's low-pass the bulk of an edge comes after its half-way point. By how
  // much is the transmitter's: 0.31 us in recordings of two, 0.13 us in those of a third. So it is measured while
  // nearly every line finds its hsync, and kept.
  static constexpr double kRefUs = 0.2, kRefMinUs = -0.1, kRefMaxUs = 0.5;
  static constexpr int kWideStep = 4;
  static constexpr uint64_t kLongMin = 2000, kLongSpan = 65536;  // lines: at least, and at most
  static constexpr int kNever = 1 << 30;

  struct Seen {
    double pos;        // where the fold has the hsync's edge, from the line's start (samples)
    float tip, blank;  // ... and its levels
  };
  int found64() const { return std::popcount(_found); }
  void fold_line(const SyncSamples &s, double ls, float sign);
  std::optional<Seen> look();
  float fold_noise() const;
  void fold_moved(double c);
  void wide_fold(const SyncSamples &s, double ls, float sign);
  std::optional<double> wide_look(double T);
  void sure(double start, uint64_t idx);
  void go_weak(const Line &l, Verdict &v);
  void enter();
  void stale();

  double _fs;
  uint64_t _found = ~0ull;  // the last 64 lines, 1: its hsync was found
  bool _weak = false;       // holding
  int _weak_lines = 0;      // lines held so far
  int _weak_ago = kNever;   // lines since the fold last held
  int _unseen = 0;          // holding: lines since the fold last showed the pulse
  uint64_t _holds = 0;
  // the fold around where each line's hsync is due
  Fold _fold;
  int _before = 0;          // it begins so many samples before the line does
  int _fold_n = 0;          // lines in it (up to kFoldLines)
  int _fold_due = 0;        // lines since it was last looked at
  float _fold_var = 0;      // a line's samples about the fold, mean square: the noise
  double _ref = 0;
  int _ref_n = 0;
  double _carry = 0;        // steering not yet passed on to the fold (less than a sample)
  bool _fold_good = false;  // not holding: the fold shows the hsync where the grid is (looked at every 16 lines)
  // the fold of the whole line
  Fold _wide;
  int _wide_n = 0;
  double _wide_at = 0;      // where the last look had an hsync, samples from the grid ...
  int64_t _wide_line = 0;   // ... and at which line (0: none)
  // The grid where it was sure (most lines found their hsync, or the fold showed it): the start of a line and
  // which line; such a line long before, and one half as long before; and the line period over the lines between
  double _sure_start = -1, _base_start = -1, _mid_start = -1;
  uint64_t _sure_idx = 0, _base_idx = 0, _mid_idx = 0;
  double _T_long = 0;
  bool _long_ok = false;
};

}  // namespace kp
