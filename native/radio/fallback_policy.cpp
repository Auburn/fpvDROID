#include "fallback_policy.h"

#include <algorithm>

#include "log.h"

namespace kp {

bool FallbackPolicy::active() const {
  std::lock_guard<std::mutex> l(_m);
  return _active;
}

bool FallbackPolicy::disabled() const {
  std::lock_guard<std::mutex> l(_m);
  return _disabled;
}

int FallbackPolicy::rate_for(int asked) const {
  std::lock_guard<std::mutex> l(_m);
  return _active && asked == 1 ? 2 : asked;
}

void FallbackPolicy::clear() {
  std::lock_guard<std::mutex> l(_m);
  _active = false;
}

void FallbackPolicy::watch_started(int64_t now_ms) {
  std::lock_guard<std::mutex> l(_m);
  _t0_ms = now_ms;
  _next_poll_ms = now_ms;
  _restarts.clear();
  _marks.clear();
}

// Evidence towards a fallback is only taken at 40 MS/s, after the grace, and while falling back is still allowed.
bool FallbackPolicy::counts(int64_t now_ms, double fs) const {
  return fs == 40e6 && !_disabled && now_ms - _t0_ms >= kGraceMs;
}

void FallbackPolicy::fall_back() {
  _active = true;
  _restarts.clear();
  _marks.clear();
}

bool FallbackPolicy::stream_restarted(int64_t now_ms, double fs) {
  std::lock_guard<std::mutex> l(_m);
  if (!counts(now_ms, fs)) return false;
  _restarts.push_back(now_ms);
  while (now_ms - _restarts.front() > kWindowMs) _restarts.pop_front();
  if (static_cast<int>(_restarts.size()) < kFaults) return false;
  KP_LOGW("radio: 40 MS/s restarted %zu times in %lld s: watching at 20 MS/s from now on", _restarts.size(),
          static_cast<long long>(kWindowMs / 1000));
  fall_back();
  return true;
}

bool FallbackPolicy::poll_due(int64_t now_ms) {
  std::lock_guard<std::mutex> l(_m);
  if (now_ms < _next_poll_ms) return false;
  _next_poll_ms = now_ms + kPollMs;
  return true;
}

bool FallbackPolicy::blocks_lost(int64_t now_ms, double fs, uint64_t lost_total) {
  std::lock_guard<std::mutex> l(_m);
  if (!counts(now_ms, fs)) {
    _marks.clear();  // the window starts with the first poll that counts: what the grace lost is not in it
    return false;
  }
  _marks.emplace_back(now_ms, lost_total);
  while (now_ms - _marks.front().first > kWindowMs) _marks.pop_front();
  const uint64_t lost = lost_total - _marks.front().second;
  if (lost < static_cast<uint64_t>(kDrops)) return false;
  KP_LOGW("radio: 40 MS/s lost %llu blocks in %.1f s: watching at 20 MS/s from now on",
          static_cast<unsigned long long>(lost), (now_ms - _marks.front().first) / 1000.0);
  fall_back();
  return true;
}

void FallbackPolicy::receiver_wedged(bool watching) {
  std::lock_guard<std::mutex> l(_m);
  if (!watching || !_active || _disabled) return;
  _disabled = true;
  KP_LOGW("radio: wedged at 20 MS/s after falling back from 40: no more falling back this session");
}

}  // namespace kp
