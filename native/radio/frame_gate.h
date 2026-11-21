// kepyur radio: torn frames are not shown.
#pragma once
#include <atomic>
#include <cstdint>
#include <limits>
#include <utility>

#include "video.h"

namespace kp {

// Sits between the decoders and the screen. After a hole in the stream (a restart, a lost block) the frame the
// decoder is in, and the next while it re-phases, are torn: those kFrames go nowhere, and the screen keeps the
// last whole one. At most once every kEveryMs: a stream with holes several times a second shows torn frames
// rather than a frozen picture (so at most 4 frames a second are hidden).
//
// gap() and the frames come from the decode thread; the rest may be called from any thread. The clock is the
// caller's (ms of one steady clock).
class FrameGate {
public:
  static constexpr int kFrames = 2;
  static constexpr int64_t kEveryMs = 500;

  explicit FrameGate(FrameSink out) : _out(std::move(out)) {}
  // What the decoders are given in place of `out`. The gate must outlive them.
  FrameSink sink() {
    return [this](const Frame &f) { frame(f); };
  }
  void frame(const Frame &f);
  // The block about to be decoded does not follow on from the one before it.
  void gap(int64_t now_ms);
  // Off: every frame is shown, torn or not (the KP_CONCEAL=0 debug switch). Frames already marked stay hidden.
  void set_enabled(bool on) { _on = on; }
  // A radio opened anew: nothing waiting to be hidden, the count from 0.
  void reset();
  uint64_t hidden() const { return _hidden; }

private:
  FrameSink _out;
  std::atomic<bool> _on{true};
  std::atomic<int> _left{0};  // frames still to hide
  std::atomic<uint64_t> _hidden{0};
  int64_t _last_ms = std::numeric_limits<int64_t>::min() / 2;  // when frames were last marked (decode thread)
};

}  // namespace kp
