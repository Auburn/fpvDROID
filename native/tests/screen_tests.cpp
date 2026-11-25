// kp_screen_tests: the screen's rules that need no GL (ctest: "screen"). Plain checks, as in radio_tests.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>

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
    m.presented(t, top, bottom, i, t + 0.010, t - 0.002);  // the compositor expects it up when it is
    m.poll(t, query);  // not shown yet
    shown[i] = static_cast<int64_t>(std::llround((t + 0.010) * 1e9));
  }
  m.poll(102, query);
  const LatencyStats s = m.take(102);
  CHECK(s.presents == 60 && s.shown == 60 && s.dropped == 0 && s.unknown == 0);
  CHECK(near(s.queued_top, 16.0) && near(s.queued_bottom, 1.0));
  CHECK(near(s.shown_top, 26.0) && near(s.shown_bottom, 11.0) && near(s.shown_top_p95, 26.0));
  CHECK(near(s.expected_top, 26.0) && near(s.expected_bottom, 11.0) && near(s.render, 2.0));
  // a new window
  const LatencyStats e = m.take(103);
  CHECK(e.presents == 0 && e.shown_top == 0 && near(e.seconds, 1.0));
}

// Strips: three presents per refresh, of which the display shows only the last (the others replaced before it
// latched them). The top line came in the first present of a refresh: it went up with the third.
static void replaced_presents_go_up_with_the_next() {
  LatencyMeter m;
  std::map<uint64_t, int64_t> shown;
  const auto query = [&](uint64_t id) { return shown.count(id) ? shown[id] : LatencyMeter::kPending; };
  m.presented(10.000, 9.998, 0, 1);  // the top strip, its top line taken 2 ms before
  m.presented(10.002, 0, 0, 2);
  m.presented(10.004, 0, 9.0035, 3);  // ... and the bottom of an older picture
  shown[1] = LatencyMeter::kDropped;
  shown[2] = LatencyMeter::kDropped;
  m.poll(10.005, query);  // 3 still pending: 1 and 2 are counted as replaced, their lines wait for a present shown
  const LatencyStats w = m.take(10.005);
  CHECK(w.shown == 0 && w.dropped == 2 && w.shown_top == 0);
  shown[3] = static_cast<int64_t>(10.012e9);
  m.poll(10.013, query);
  const LatencyStats s = m.take(10.013);
  CHECK(s.shown == 1 && s.dropped == 0);
  CHECK(near(s.shown_top, 14.0));      // taken 9.998, up at 10.012
  CHECK(near(s.shown_bottom, 1008.5));  // the bottom in present 3 itself
  // a present the display never says anything of is given up after a while, and the next one does not take its
  // lines
  m.presented(11.000, 10.990, 0, 4);
  m.presented(11.002, 0, 0, 5);
  shown[5] = static_cast<int64_t>(11.010e9);
  m.poll(11.003, query);
  CHECK(m.take(11.003).shown == 0);
  m.poll(11.000 + LatencyMeter::kGiveUpS + 0.01, query);
  const LatencyStats g = m.take(12);
  CHECK(g.unknown == 1 && g.shown == 1 && g.shown_top == 0);
  // a present with no id (the display gives no times) is counted, and never waited for
  m.presented(12.5, 12.49, 12.495, 0);
  m.poll(12.5, query);
  const LatencyStats n = m.take(13);
  CHECK(n.presents == 1 && n.shown == 0 && near(n.queued_top, 10.0));
}

int main() {
  whole_fields();
  replaced_presents_go_up_with_the_next();
  std::fprintf(stderr, "kp_screen_tests: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
