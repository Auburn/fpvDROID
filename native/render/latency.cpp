// fpvDROID screen: picture delay. See latency.h.
#include "latency.h"

#include <algorithm>
#include <cstdio>

namespace kp {

namespace {
float ms_between(double from, double to) { return static_cast<float>((to - from) * 1e3); }

// The value a fraction q of the way up v (0.5: the median); 0 for none. Reorders v.
float quantile(std::vector<float> &v, double q) {
  if (v.empty()) return 0;
  const size_t k = std::min(v.size() - 1, static_cast<size_t>(q * static_cast<double>(v.size())));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}

// ... the median, and the samples gone
float take_median(std::vector<float> &v) {
  const float m = quantile(v, 0.5);
  v.clear();
  return m;
}
}  // namespace

void LatencyMeter::Samples::add(double from_top, double from_bottom, double to) {
  if (from_top > 0) top.push_back(ms_between(from_top, to));
  if (from_bottom > 0) bottom.push_back(ms_between(from_bottom, to));
}

Ends LatencyMeter::Samples::medians() { return Ends{take_median(top), take_median(bottom)}; }

void LatencyMeter::presented(const Present &p) {
  if (p.swap_ms > 0) _swap.push_back(p.swap_ms);
  if (_t0 <= 0) _t0 = p.t;
  _presents++;
  _handed.add(p.top_t, p.bottom_t, p.t);
  if (p.expected > 0) _expected.add(p.top_t, p.bottom_t, p.expected);
  if (p.posted > 0) _render.push_back(ms_between(p.posted, p.t));
  if (p.id) _wait.push_back(Waiting{p.id, p.t, p.top_t, p.bottom_t});
  // nobody polls (the display gives no times): keep it from growing
  while (_wait.size() > 4096) _wait.pop_front();
}

void LatencyMeter::poll(double now, const std::function<int64_t(uint64_t)> &query) {
  while (!_wait.empty()) {
    const Waiting &p = _wait.front();
    const int64_t v = query(p.id);
    if (v == kPending && now - p.t < kGiveUpS) break;
    if (v == kDropped) {
      // its lines go up with a later present: the newest picture top / bottom wins (an older one is never shown)
      if (p.top_t > 0) _carry_top = p.top_t;
      if (p.bottom_t > 0) _carry_bottom = p.bottom_t;
      _dropped++;
    } else if (v >= 0) {
      _displayed.add(p.top_t > 0 ? p.top_t : _carry_top, p.bottom_t > 0 ? p.bottom_t : _carry_bottom,
                     static_cast<double>(v) * 1e-9);
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

LatencyStats LatencyMeter::take(double now, float refresh_hz, bool front) {
  LatencyStats s;
  s.seconds = _t0 > 0 ? static_cast<float>(now - _t0) : 0.f;
  s.presents = _presents;
  s.shown = _shown;
  s.dropped = _dropped;
  s.unknown = _unknown;
  s.handed = _handed.medians();
  s.displayed_top_p95 = quantile(_displayed.top, 0.95);
  s.displayed = _displayed.medians();
  s.expected = _expected.medians();
  s.to_screen = take_median(_to_screen);
  s.render = take_median(_render);
  s.swap = take_median(_swap);
  s.refresh_hz = refresh_hz;
  s.front = front;
  _presents = _shown = _dropped = _unknown = 0;
  _t0 = now;
  return s;
}

void LatencyMeter::forget() {
  _wait.clear();
  _carry_top = _carry_bottom = 0;
}

std::string latency_json(const LatencyStats &s) {
  const float per_s = s.seconds > 0 ? 1 / s.seconds : 0.f;
  char b[400];
  std::snprintf(b, sizeof b,
                "{\"presents_s\":%.0f,\"shown_s\":%.0f,\"queued_top_ms\":%.1f,\"queued_bottom_ms\":%.1f,"
                "\"shown_top_ms\":%.1f,\"shown_bottom_ms\":%.1f,\"shown_top_p95_ms\":%.1f,"
                "\"expected_top_ms\":%.1f,\"expected_bottom_ms\":%.1f,\"refresh_hz\":%.1f,\"front\":%s}",
                s.presents * per_s, s.shown * per_s, s.handed.top, s.handed.bottom, s.displayed.top,
                s.displayed.bottom, s.displayed_top_p95, s.expected.top, s.expected.bottom, s.refresh_hz,
                s.front ? "true" : "false");
  return b;
}

std::string latency_log(const LatencyStats &s) {
  char b[600];
  std::snprintf(b, sizeof b,
                "screen%s: %d presents in %.1f s (%d shown, %d replaced, %d not known), display %.0f Hz; picture taken "
                "-> handed over: top %.1f ms, bottom %.1f ms; -> shown: top %.1f ms (95%% %.1f), bottom %.1f ms; -> "
                "expected shown: top %.1f ms, bottom %.1f ms; to the screen %.1f ms, render %.1f ms (swap %.1f)",
                s.front ? " (front buffer: handed over = drawn on screen)" : "", s.presents, s.seconds, s.shown,
                s.dropped, s.unknown, s.refresh_hz, s.handed.top, s.handed.bottom, s.displayed.top,
                s.displayed_top_p95, s.displayed.bottom, s.expected.top, s.expected.bottom, s.to_screen, s.render,
                s.swap);
  return b;
}

}  // namespace kp
