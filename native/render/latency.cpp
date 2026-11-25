// fpvDROID screen: picture delay. See latency.h.
#include "latency.h"

#include <algorithm>

namespace kp {

void LatencyMeter::presented(double t, double top_t, double bottom_t, uint64_t id, double expected, double posted,
                             float swap_ms) {
  if (swap_ms > 0) _swap.push_back(swap_ms);
  if (_t0 <= 0) _t0 = t;
  _presents++;
  if (top_t > 0) _q_top.push_back(static_cast<float>((t - top_t) * 1e3));
  if (bottom_t > 0) _q_bottom.push_back(static_cast<float>((t - bottom_t) * 1e3));
  if (expected > 0 && top_t > 0) _e_top.push_back(static_cast<float>((expected - top_t) * 1e3));
  if (expected > 0 && bottom_t > 0) _e_bottom.push_back(static_cast<float>((expected - bottom_t) * 1e3));
  if (posted > 0) _render.push_back(static_cast<float>((t - posted) * 1e3));
  if (id) _wait.push_back(Present{id, t, top_t, bottom_t});
  // nobody polls (the display gives no times): keep it from growing
  while (_wait.size() > 4096) _wait.pop_front();
}

void LatencyMeter::poll(double now, const std::function<int64_t(uint64_t)> &query) {
  while (!_wait.empty()) {
    const Present &p = _wait.front();
    const int64_t v = query(p.id);
    if (v == kPending && now - p.t < kGiveUpS) break;
    if (v == kDropped) {
      // its lines go up with a later present: the newest picture top / bottom wins (an older one is never shown)
      if (p.top_t > 0) _carry_top = p.top_t;
      if (p.bottom_t > 0) _carry_bottom = p.bottom_t;
      _dropped++;
    } else if (v >= 0) {
      const double at = static_cast<double>(v) * 1e-9;
      const double top = p.top_t > 0 ? p.top_t : _carry_top, bottom = p.bottom_t > 0 ? p.bottom_t : _carry_bottom;
      if (top > 0) _s_top.push_back(static_cast<float>((at - top) * 1e3));
      if (bottom > 0) _s_bottom.push_back(static_cast<float>((at - bottom) * 1e3));
      _carry_top = _carry_bottom = 0;
      _shown++;
    } else {
      _unknown++;  // given up, or the display cannot say: its lines' times go with it
    }
    _wait.pop_front();
  }
}

void LatencyMeter::handed(const std::vector<float> &ages_ms) {
  _to_screen.insert(_to_screen.end(), ages_ms.begin(), ages_ms.end());
}

float LatencyMeter::median(std::vector<float> &v, double q) {
  if (v.empty()) return 0;
  const size_t k = std::min(v.size() - 1, static_cast<size_t>(q * static_cast<double>(v.size())));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}

LatencyStats LatencyMeter::take(double now) {
  LatencyStats s;
  s.seconds = _t0 > 0 ? static_cast<float>(now - _t0) : 0.f;
  s.presents = _presents;
  s.shown = _shown;
  s.dropped = _dropped;
  s.unknown = _unknown;
  s.queued_top = median(_q_top);
  s.queued_bottom = median(_q_bottom);
  s.shown_top_p95 = median(_s_top, 0.95);
  s.shown_top = median(_s_top);
  s.shown_bottom = median(_s_bottom);
  s.expected_top = median(_e_top);
  s.expected_bottom = median(_e_bottom);
  s.to_screen = median(_to_screen);
  s.render = median(_render);
  s.swap = median(_swap);
  for (auto *v : {&_q_top, &_q_bottom, &_s_top, &_s_bottom, &_e_top, &_e_bottom, &_to_screen, &_render, &_swap}) v->clear();
  _presents = _shown = _dropped = _unknown = 0;
  _t0 = now;
  return s;
}

void LatencyMeter::forget() {
  _wait.clear();
  _carry_top = _carry_bottom = 0;
}

}  // namespace kp
