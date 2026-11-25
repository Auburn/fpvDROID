#include "frame_gate.h"

namespace kp {

void FrameGate::frame(const Frame &f) {
  if (_left.load(std::memory_order_relaxed) > 0) {
    _left--;
    _hidden++;
    return;
  }
  _out(f);
}

void FrameGate::strip(const Strip &s) {
  if (_left.load(std::memory_order_relaxed) > 0 || !_strips) return;  // in a frame that is to be hidden
  _strips(s);
}

void FrameGate::gap(int64_t now_ms) {
  if (!_on || now_ms - _last_ms < kEveryMs) return;
  _left = kFrames;
  _last_ms = now_ms;
}

void FrameGate::reset() {
  _left = 0;
  _hidden = 0;
}

}  // namespace kp
