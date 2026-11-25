#include "fallback_policy.h"

#include <algorithm>

#include "log.h"

namespace kp {

bool FallbackPolicy::active() const {
  std::lock_guard<std::mutex> l(_m);
  return _fb != Fallback::None;
}

bool FallbackPolicy::disabled() const {
  std::lock_guard<std::mutex> l(_m);
  return _disabled;
}

int FallbackPolicy::rate_for(int asked) const {
  std::lock_guard<std::mutex> l(_m);
  return _fb != Fallback::None && asked == 1 ? 2 : asked;
}

void FallbackPolicy::adapter_opened() {
  std::lock_guard<std::mutex> l(_m);
  if (_fb == Fallback::Evidence) _fb = Fallback::None;
}

void FallbackPolicy::watch_rate_set() {
  std::lock_guard<std::mutex> l(_m);
  _fb = Fallback::None;
}

uint64_t FallbackPolicy::late_recent(int64_t now_ms) const {
  std::lock_guard<std::mutex> l(_m);
  if (!_watching || _first_ms < 0) return 0;
  // from the earliest count that is both past the stream's first moments and within the window: the polls have
  // kept only the last kWindowMs as of the latest one, and the time since then takes more out
  for (const auto &m : _marks)
    if (m.first - _first_ms >= kSettleMs && now_ms - m.first <= kWindowMs) return _marks.back().second - m.second;
  return 0;
}

void FallbackPolicy::watch_started(int64_t now_ms) {
  std::lock_guard<std::mutex> l(_m);
  _watching = true;
  _t0_ms = now_ms;
  _next_poll_ms = now_ms;  // the first block of the watch gives the count to measure from
  _first_ms = -1;
  _restarts.clear();
  _marks.clear();
}

void FallbackPolicy::watch_ended() {
  std::lock_guard<std::mutex> l(_m);
  _watching = false;
  _first_ms = -1;
  _restarts.clear();
  _marks.clear();
}

// Evidence towards a fallback is only taken at 40 MS/s, after the grace, and while falling back is still allowed.
bool FallbackPolicy::counts(int64_t now_ms, double fs) const {
  return fs == 40e6 && !_disabled && now_ms - _t0_ms >= kGraceMs;
}

void FallbackPolicy::fall_back(Fallback to) {
  if (_fb != Fallback::Held) _fb = to;  // a held fallback stays held, whatever else is seen
  _restarts.clear();
  _marks.clear();  // the 20 MS/s stream is measured from its own first block
  _first_ms = -1;
}

bool FallbackPolicy::stream_restarted(int64_t now_ms, double fs) {
  std::lock_guard<std::mutex> l(_m);
  if (!counts(now_ms, fs)) return false;
  _restarts.push_back(now_ms);
  while (now_ms - _restarts.front() > kWindowMs) _restarts.pop_front();
  if (static_cast<int>(_restarts.size()) < kFaults) return false;
  KP_LOGW("radio: 40 MS/s restarted %zu times in %lld s: watching at 20 MS/s from now on", _restarts.size(),
          static_cast<long long>(kWindowMs / 1000));
  fall_back(Fallback::Evidence);
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
  if (_first_ms < 0) _first_ms = now_ms;
  _marks.emplace_back(now_ms, lost_total);
  while (now_ms - _marks.front().first > kWindowMs) _marks.pop_front();
  if (!counts(now_ms, fs)) return false;
  // from the first poll after the grace: what the grace itself lost is not held against the stream
  const auto from = std::find_if(_marks.begin(), _marks.end(),
                                 [&](const auto &m) { return m.first - _t0_ms >= kGraceMs; });
  const uint64_t lost = lost_total - from->second;
  if (lost < static_cast<uint64_t>(kDrops)) return false;
  KP_LOGW("radio: 40 MS/s lost %llu blocks in %.1f s: watching at 20 MS/s from now on",
          static_cast<unsigned long long>(lost), (now_ms - from->first) / 1000.0);
  fall_back(Fallback::Evidence);
  return true;
}

void FallbackPolicy::receiver_stopped(Stopped why, bool watching, double fs) {
  std::lock_guard<std::mutex> l(_m);
  if (!watching || _disabled) return;
  if (why == Stopped::Wedged && _fb == Fallback::Evidence) {
    // not after a held fallback: there 20 MS/s is not the worse of the two, it is all there is
    _disabled = true;
    KP_LOGW("radio: wedged at 20 MS/s after falling back from 40: no more falling back this session");
  } else if (why == Stopped::StreamFailing && fs == 40e6 && _fb == Fallback::None) {
    KP_LOGW("radio: 40 MS/s would not stream (the receiver gave up): watching at 20 MS/s from now on, also when the "
            "adapter is opened again");
    fall_back(Fallback::Held);
  }
}

}  // namespace kp
