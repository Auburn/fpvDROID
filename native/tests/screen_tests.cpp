// kp_screen_tests: the screen's rules that need no GL (ctest: "screen"). Plain checks, as in radio_tests.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>

#include "latency.h"

using namespace kp;

static int g_checks = 0, g_failed = 0;
#define CHECK(c)                                                                  \
  do {                                                                            \
    g_checks++;                                                                   \
    if (!(c)) {                                                                   \
      g_failed++;                                                                 \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c);  \
    }                                                                             \
  } while (0)

static bool near(float a, double b) { return std::fabs(a - b) < 0.01; }

// Whole fields: a present each, the picture's top line taken a field before its bottom; the display shows each
// present 10 ms after it was handed over.
static void whole_fields() {
  LatencyMeter m;
  std::map<uint64_t, int64_t> shown;  // what the display says of each present
  const auto query = [&](uint64_t id) {
    const auto it = shown.find(id);
    return it == shown.end() ? LatencyMeter::kPending : it->second;
  };
  const double field = 1 / 59.94;
  for (int i = 1; i <= 60; i++) {
    const double t = 100 + i * field, bottom = t - 0.001, top = bottom - 0.015;
    // the compositor expects it up when it is
    m.presented({.t = t, .top_t = top, .bottom_t = bottom, .id = static_cast<uint64_t>(i), .expected = t + 0.010,
                 .posted = t - 0.002});
    m.poll(t, query);  // not shown yet
    shown[i] = static_cast<int64_t>(std::llround((t + 0.010) * 1e9));
  }
  m.poll(102, query);
  const LatencyStats s = m.take(102);
  CHECK(s.presents == 60 && s.shown == 60 && s.dropped == 0 && s.unknown == 0);
  CHECK(near(s.handed.top, 16.0) && near(s.handed.bottom, 1.0));
  CHECK(near(s.displayed.top, 26.0) && near(s.displayed.bottom, 11.0) && near(s.displayed_top_p95, 26.0));
  CHECK(near(s.expected.top, 26.0) && near(s.expected.bottom, 11.0) && near(s.render, 2.0));
  // a new window
  const LatencyStats e = m.take(103);
  CHECK(e.presents == 0 && e.displayed.top == 0 && near(e.seconds, 1.0));
}

// Strips: three presents per refresh, of which the display shows only the last (the others replaced before it
// latched them). The top line came in the first present of a refresh: it went up with the third.
static void replaced_presents_go_up_with_the_next() {
  LatencyMeter m;
  std::map<uint64_t, int64_t> shown;
  const auto query = [&](uint64_t id) { return shown.count(id) ? shown[id] : LatencyMeter::kPending; };
  m.presented({.t = 10.000, .top_t = 9.998, .id = 1});  // the top strip, its top line taken 2 ms before
  m.presented({.t = 10.002, .id = 2});
  m.presented({.t = 10.004, .bottom_t = 9.0035, .id = 3});  // ... and the bottom of an older picture
  shown[1] = LatencyMeter::kDropped;
  shown[2] = LatencyMeter::kDropped;
  m.poll(10.005, query);  // 3 still pending: 1 and 2 are counted as replaced, their lines wait for a present shown
  const LatencyStats w = m.take(10.005);
  CHECK(w.shown == 0 && w.dropped == 2 && w.displayed.top == 0);
  shown[3] = static_cast<int64_t>(10.012e9);
  m.poll(10.013, query);
  const LatencyStats s = m.take(10.013);
  CHECK(s.shown == 1 && s.dropped == 0);
  CHECK(near(s.displayed.top, 14.0));       // taken 9.998, up at 10.012
  CHECK(near(s.displayed.bottom, 1008.5));  // the bottom in present 3 itself
  // a present the display never says anything of is given up after a while, and the next one does not take its
  // lines
  m.presented({.t = 11.000, .top_t = 10.990, .id = 4});
  m.presented({.t = 11.002, .id = 5});
  shown[5] = static_cast<int64_t>(11.010e9);
  m.poll(11.003, query);
  CHECK(m.take(11.003).shown == 0);
  m.poll(11.000 + LatencyMeter::kGiveUpS + 0.01, query);
  const LatencyStats g = m.take(12);
  CHECK(g.unknown == 1 && g.shown == 1 && g.displayed.top == 0);
  // a present with no id (the display gives no times) is counted, and never waited for
  m.presented({.t = 12.5, .top_t = 12.49, .bottom_t = 12.495});
  m.poll(12.5, query);
  const LatencyStats n = m.take(13);
  CHECK(n.presents == 1 && n.shown == 0 && near(n.handed.top, 10.0));
}

// The window as the app reads it (the keys are RadioStatus.kt's PictureDelay), and as the log has it. Front-buffer
// drawing: two presents a second apart, each drawn 10 ms after its top line was taken and 6 ms after its bottom one,
// expected on the display 4 ms on; the display gives no times.
static void the_window_for_the_app_and_the_log() {
  LatencyMeter m;
  m.presented({.t = 5.010, .top_t = 5.000, .bottom_t = 5.004, .expected = 5.014});
  m.presented({.t = 5.510, .top_t = 5.500, .bottom_t = 5.504, .expected = 5.514});
  const LatencyStats s = m.take(6.010, 120, true);
  CHECK(s.refresh_hz == 120 && s.front && near(s.seconds, 1.0));
  const std::string json = latency_json(s);
  CHECK(json == "{\"presents_s\":2,\"shown_s\":0,\"queued_top_ms\":10.0,\"queued_bottom_ms\":6.0,\"shown_top_ms\":0.0,"
                "\"shown_bottom_ms\":0.0,\"shown_top_p95_ms\":0.0,\"expected_top_ms\":14.0,\"expected_bottom_ms\":10.0,"
                "\"refresh_hz\":120.0,\"front\":true}");
  if (g_failed) std::fprintf(stderr, "  %s\n", json.c_str());
  const std::string log = latency_log(s);
  CHECK(log.find("screen (front buffer: handed over = drawn on screen): 2 presents in 1.0 s") == 0);
  CHECK(log.find("display 120 Hz") != std::string::npos);
  CHECK(log.find("expected shown: top 14.0 ms, bottom 10.0 ms") != std::string::npos);
  // nothing in the window, the display not known: all noughts, and no front buffer
  const std::string none = latency_json(m.take(7));
  CHECK(none.find("\"presents_s\":0,") != std::string::npos);
  CHECK(none.find("\"refresh_hz\":0.0,\"front\":false}") != std::string::npos);
}

int main() {
  whole_fields();
  replaced_presents_go_up_with_the_next();
  the_window_for_the_app_and_the_log();
  std::fprintf(stderr, "kp_screen_tests: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
