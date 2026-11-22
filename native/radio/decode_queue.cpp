#include "decode_queue.h"

#include <chrono>
#include <utility>

namespace kp {

DemodBlock DecodeQueue::blank() {
  DemodBlock b;
  std::lock_guard<std::mutex> l(_m);
  if (!_pool.empty()) {
    b.d = std::move(_pool.back().d);
    b.bad = std::move(_pool.back().bad);
    _pool.pop_back();
  }
  return b;
}

void DecodeQueue::keep(DemodBlock &b) {
  if (_pool.size() < kPool) _pool.push_back(Buffers{std::move(b.d), std::move(b.bad)});
}

void DecodeQueue::push(DemodBlock &&b) {
  {
    std::lock_guard<std::mutex> l(_m);
    if (_q.size() >= kCap) {
      const bool carried = _q.front().gap;
      keep(_q.front());
      _q.pop_front();
      _dropped++;
      if (carried && !_q.empty()) _q.front().gap = true;
    }
    _q.push_back(std::move(b));
  }
  _cv.notify_one();
}

bool DecodeQueue::pop(DemodBlock &out, int wait_ms, const std::atomic<bool> &stop) {
  std::unique_lock<std::mutex> l(_m);
  _cv.wait_for(l, std::chrono::milliseconds(wait_ms), [&] { return !_q.empty() || stop; });
  if (_q.empty()) return false;
  out = std::move(_q.front());
  _q.pop_front();
  return true;
}

void DecodeQueue::recycle(DemodBlock &&b) {
  std::lock_guard<std::mutex> l(_m);
  keep(b);
}

void DecodeQueue::wake() { _cv.notify_all(); }

void DecodeQueue::clear() {
  std::lock_guard<std::mutex> l(_m);
  _q.clear();
}

}  // namespace kp
