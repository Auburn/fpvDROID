// kp_radio_tests: the radio's rules that need no adapter (ctest: "radio"). Plain checks, no framework: each
// failed CHECK prints its line, and the program's exit status says whether any did.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "decode_queue.h"
#include "fallback_policy.h"
#include "frame_gate.h"

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

using FP = FallbackPolicy;
static constexpr double k40 = 40e6, k20 = 20e6;

// A watch from t0 to t1 with the receiver's count read whenever the policy asks: `total` at every poll. True
// when a poll said to fall back.
static bool poll(FP &p, int64_t t0, int64_t t1, double fs, uint64_t total) {
  for (int64_t t = t0; t <= t1; t += 50)
    if (p.poll_due(t) && p.blocks_lost(t, fs, total)) return true;
  return false;
}

// ---------------------------------------------------------------------------------------- FallbackPolicy

static void fallback_restarts() {
  FP p;
  CHECK(!p.active() && p.rate_for(1) == 1 && p.rate_for(2) == 2);
  p.watch_started(1000);
  // any number of them inside the grace: a stream that starts badly is left alone
  for (int64_t t = 1000; t < 1000 + FP::kGraceMs; t += 300) CHECK(!p.stream_restarted(t, k40));
  CHECK(!p.active());
  // after it, the third within the window
  const int64_t g = 1000 + FP::kGraceMs;
  CHECK(!p.stream_restarted(g, k40));
  CHECK(!p.stream_restarted(g + 2000, k40));
  CHECK(p.stream_restarted(g + FP::kWindowMs, k40));  // the first is still (just) inside the window
  CHECK(p.active() && p.rate_for(1) == 2 && p.rate_for(2) == 2);
}

static void fallback_restarts_spread_out() {
  FP p;
  p.watch_started(0);
  // two in any window, for a minute: a stream that restarts now and then stays at 40 MS/s
  for (int64_t t = FP::kGraceMs; t < 60000; t += FP::kWindowMs / 2 + 1) CHECK(!p.stream_restarted(t, k40));
  CHECK(!p.active());
}

static void fallback_only_from_40() {
  FP p;
  p.watch_started(0);
  for (int64_t t = FP::kGraceMs; t < FP::kGraceMs + 3000; t += 100) CHECK(!p.stream_restarted(t, k20));
  CHECK(!poll(p, FP::kGraceMs + 3000, FP::kGraceMs + 6000, k20, 0));
  CHECK(!poll(p, FP::kGraceMs + 6050, FP::kGraceMs + 9000, k20, 500));
  CHECK(!p.active());
}

static void fallback_lost_blocks() {
  FP p;
  p.watch_started(0);
  // lost during the grace: not held against the stream, however many
  CHECK(!poll(p, 0, 5000, k40, 7));
  CHECK(!poll(p, 5050, FP::kGraceMs + 1000, k40, 7 + 400));
  // one short of the threshold after it: still not
  CHECK(!poll(p, FP::kGraceMs + 1050, FP::kGraceMs + 2000, k40, 407 + FP::kDrops - 1));
  CHECK(!p.active());
  // the one more
  CHECK(poll(p, FP::kGraceMs + 2050, FP::kGraceMs + 3000, k40, 407 + FP::kDrops));
  CHECK(p.active());
}

static void fallback_lost_blocks_slowly() {
  FP p;
  p.watch_started(0);
  // the threshold less one in every window, kept up: a host that stalls now and then stays at 40 MS/s
  uint64_t total = 0;
  for (int64_t t = FP::kGraceMs; t < 90000; t += FP::kWindowMs + FP::kPollMs + 50) {
    total += FP::kDrops - 1;
    CHECK(!poll(p, t, t + FP::kWindowMs + FP::kPollMs, k40, total));
  }
  CHECK(!p.active());
}

static void fallback_poll_cadence() {
  FP p;
  p.watch_started(100);
  CHECK(p.poll_due(100));  // the watch's first block gives the count to measure from
  CHECK(!p.poll_due(100 + FP::kPollMs - 1));
  CHECK(p.poll_due(100 + FP::kPollMs));
  CHECK(!p.poll_due(100 + FP::kPollMs + 1));
}

static void fallback_cleared() {
  FP p;
  p.watch_started(0);
  for (int i = 0; i < FP::kFaults; i++) p.stream_restarted(FP::kGraceMs + i, k40);
  CHECK(p.active());
  // Radio::open() and a changed Radio::set_watch_rate(): 40 MS/s is tried again ...
  p.clear();
  CHECK(!p.active() && !p.disabled() && p.rate_for(1) == 1);
  // ... and can fall back again
  p.watch_started(100000);
  bool fell = false;
  for (int i = 0; i < FP::kFaults; i++) fell = p.stream_restarted(100000 + FP::kGraceMs + i, k40);
  CHECK(fell && p.active());
}

static void fallback_disabled_by_a_wedge_at_20() {
  FP p;
  p.watch_started(0);
  // a wedge at 40 MS/s, or outside a watch, says nothing about 20 MS/s
  p.receiver_wedged(true);
  CHECK(!p.disabled());
  for (int i = 0; i < FP::kFaults; i++) p.stream_restarted(FP::kGraceMs + i, k40);
  CHECK(p.active());
  p.receiver_wedged(false);
  CHECK(!p.disabled());
  // fallen back, watching, wedged: 20 MS/s is the worse of the two here
  p.receiver_wedged(true);
  CHECK(p.disabled() && p.active());
  // the reopen clears the fallback and keeps the lesson: no evidence of either kind counts from now on
  p.clear();
  CHECK(p.disabled() && !p.active() && p.rate_for(1) == 1);
  p.watch_started(100000);
  const int64_t g = 100000 + FP::kGraceMs;
  for (int64_t t = g; t < g + 2000; t += 50) CHECK(!p.stream_restarted(t, k40));
  CHECK(!poll(p, g + 2000, g + 3000, k40, 0));
  CHECK(!poll(p, g + 3050, g + 6000, k40, 1000));
  CHECK(!p.active());
}

static void late_recent_counts_the_current_watch() {
  FP p;
  CHECK(p.late_recent(0) == 0);  // no watch
  // what the count stood at before the watch (a scan, a stream test, an earlier watch) is not this watch's
  p.watch_started(1000);
  CHECK(!poll(p, 1000, 2000, k20, 50));
  CHECK(p.late_recent(2000) == 0);
  // at any rate, and inside the grace: three more
  CHECK(!poll(p, 2050, 3000, k20, 53));
  CHECK(p.late_recent(3000) == 3);
  // they stay for the window, then go
  CHECK(!poll(p, 3050, 2000 + FP::kWindowMs, k20, 53));
  CHECK(p.late_recent(2000 + FP::kWindowMs) == 3);
  CHECK(!poll(p, 2050 + FP::kWindowMs, 3000 + FP::kWindowMs, k20, 53));
  CHECK(p.late_recent(3000 + FP::kWindowMs) == 0);
  // another mode: nothing
  CHECK(!poll(p, 3050 + FP::kWindowMs, 20000, k20, 53));
  CHECK(!poll(p, 20050, 21000, k20, 60));
  CHECK(p.late_recent(21000) == 7);
  p.watch_ended();
  CHECK(p.late_recent(21000) == 0);
  // the next watch starts from the count as it then stands
  p.watch_started(30000);
  CHECK(!poll(p, 30000, 31000, k40, 90));
  CHECK(p.late_recent(31000) == 0);
  CHECK(!poll(p, 31050, 32000, k40, 91));
  CHECK(p.late_recent(32000) == 1);
}

// One poll at t: the receiver's count stands at `total`.
static void count_at(FP &p, int64_t t, uint64_t total) {
  CHECK(p.poll_due(t));
  CHECK(!p.blocks_lost(t, k40, total));
}

static void late_recent_leaves_out_a_streams_first_moments() {
  FP p;
  p.watch_started(0);
  // the stream's first block comes 100 ms into the watch; 11 ms later it loses four (a Pixel 8 Pro, every start)
  count_at(p, 100, 20);
  CHECK(p.late_recent(100) == 0);
  count_at(p, 300, 24);
  CHECK(p.late_recent(300) == 0);
  // settled: counted from here
  count_at(p, 500, 24);
  CHECK(p.late_recent(500) == 0);
  count_at(p, 700, 25);
  CHECK(p.late_recent(700) == 1);
  count_at(p, 900, 27);
  CHECK(p.late_recent(900) == 3);
  // long after, the start no longer matters: only the window does
  for (int64_t t = 1100; t <= 20000; t += FP::kPollMs) count_at(p, t, 27);
  CHECK(p.late_recent(20000) == 0);
  count_at(p, 20200, 30);
  CHECK(p.late_recent(20200) == 3);
  // and the same at the start of the next watch
  p.watch_ended();
  p.watch_started(30000);
  count_at(p, 30050, 30);
  count_at(p, 30250, 34);
  CHECK(p.late_recent(30250) == 0);
  count_at(p, 30450, 34);
  count_at(p, 30650, 35);
  CHECK(p.late_recent(30650) == 1);
}

static void late_recent_goes_when_the_stream_stops() {
  FP p;
  p.watch_started(0);
  CHECK(!poll(p, 0, 1000, k40, 0));
  CHECK(!poll(p, 1050, 2000, k40, 4));
  CHECK(p.late_recent(2000) == 4);
  // no more blocks, so no more polls: the last count ages out by the clock alone
  CHECK(p.late_recent(1000 + FP::kWindowMs) == 4);
  CHECK(p.late_recent(2100 + FP::kWindowMs) == 0);
  CHECK(p.late_recent(60000) == 0);
}

static void late_recent_starts_over_at_a_fallback() {
  FP p;
  p.watch_started(0);
  CHECK(!poll(p, 0, FP::kGraceMs + 500, k40, 0));
  CHECK(poll(p, FP::kGraceMs + 550, FP::kGraceMs + 1500, k40, FP::kDrops));
  CHECK(p.active());
  // the 20 MS/s stream is measured from its own first block
  const int64_t t = FP::kGraceMs + 2000;
  CHECK(p.late_recent(t) == 0);
  CHECK(!poll(p, t, t + 1000, k20, FP::kDrops + 5));
  CHECK(p.late_recent(t + 1000) == 0);
  CHECK(!poll(p, t + 1050, t + 2000, k20, FP::kDrops + 7));
  CHECK(p.late_recent(t + 2000) == 2);
}

// ---------------------------------------------------------------------------------------- FrameGate

static void gate_hides_the_frames_after_a_gap() {
  int shown = 0;
  FrameGate g([&](const Frame &) { shown++; });
  const FrameSink in = g.sink();
  const Frame f{nullptr, 0, 0, nullptr};
  in(f);
  in(f);
  CHECK(shown == 2 && g.hidden() == 0);
  g.gap(1000);
  for (int i = 0; i < FrameGate::kFrames; i++) in(f);
  CHECK(shown == 2 && g.hidden() == FrameGate::kFrames);
  in(f);
  CHECK(shown == 3);
}

static void gate_hides_at_most_so_often() {
  int shown = 0;
  FrameGate g([&](const Frame &) { shown++; });
  const Frame f{nullptr, 0, 0, nullptr};
  g.gap(1000);
  g.frame(f);
  // more gaps close behind: the frames already marked go, no new ones are
  g.gap(1100);
  g.gap(1000 + FrameGate::kEveryMs - 1);
  g.frame(f);
  g.frame(f);
  CHECK(shown == 1 && g.hidden() == FrameGate::kFrames);
  g.gap(1000 + FrameGate::kEveryMs);
  g.frame(f);
  CHECK(shown == 1 && g.hidden() == FrameGate::kFrames + 1);
}

static void gate_switched_off_and_reset() {
  int shown = 0;
  FrameGate g([&](const Frame &) { shown++; });
  const Frame f{nullptr, 0, 0, nullptr};
  g.set_enabled(false);
  g.gap(1000);
  g.frame(f);
  CHECK(shown == 1 && g.hidden() == 0);
  g.set_enabled(true);
  g.gap(2000);
  g.frame(f);
  CHECK(shown == 1 && g.hidden() == 1);
  // a radio opened anew: the frame still marked is shown, the count starts over
  g.reset();
  g.frame(f);
  CHECK(shown == 2 && g.hidden() == 0);
}

// ---------------------------------------------------------------------------------------- DecodeQueue

static DemodBlock block(DecodeQueue &q, float tag, bool gap = false) {
  DemodBlock b = q.blank();
  b.d.assign(4, tag);
  b.gap = gap;
  return b;
}

static void queue_in_order() {
  DecodeQueue q;
  const std::atomic<bool> stop{false};
  DemodBlock b;
  CHECK(!q.pop(b, 1, stop));
  q.push(block(q, 1));
  q.push(block(q, 2, true));
  CHECK(q.pop(b, 1, stop) && b.d[0] == 1 && !b.gap);
  CHECK(q.pop(b, 1, stop) && b.d[0] == 2 && b.gap);
  CHECK(!q.pop(b, 1, stop) && q.dropped() == 0);
}

static void queue_full_loses_the_oldest_and_keeps_its_gap() {
  DecodeQueue q;
  const std::atomic<bool> stop{false};
  q.push(block(q, 0, true));  // the stream had a hole before this one
  for (size_t i = 1; i <= DecodeQueue::kCap; i++) q.push(block(q, static_cast<float>(i)));
  CHECK(q.dropped() == 1);
  DemodBlock b;
  // block 0 went; the hole it marked is now marked on block 1
  CHECK(q.pop(b, 1, stop) && b.d[0] == 1 && b.gap);
  CHECK(q.pop(b, 1, stop) && b.d[0] == 2 && !b.gap);
  // lost blocks without a gap mark nothing on the next (two places were free: the rest of these each lose one)
  for (size_t i = 0; i < DecodeQueue::kCap; i++) q.push(block(q, 100));
  CHECK(q.dropped() == 1 + DecodeQueue::kCap - 2);
  CHECK(q.pop(b, 1, stop) && !b.gap);
  q.reset_dropped();
  CHECK(q.dropped() == 0);
  q.clear();
  CHECK(!q.pop(b, 1, stop));
}

static void queue_buffers_go_round() {
  DecodeQueue q;
  const std::atomic<bool> stop{false};
  DemodBlock b = q.blank();
  b.d.reserve(5000);
  b.bad.reserve(6000);
  q.push(std::move(b));
  DemodBlock got;
  CHECK(q.pop(got, 1, stop));
  q.recycle(std::move(got));
  // the pair comes back together, with the room it had
  const DemodBlock again = q.blank();
  CHECK(again.d.capacity() >= 5000 && again.bad.capacity() >= 6000 && !again.fe && !again.gap);
  CHECK(q.blank().d.capacity() == 0);
}

static void queue_pop_gives_up_when_told_to_stop() {
  DecodeQueue q;
  const std::atomic<bool> stop{true};
  DemodBlock b;
  CHECK(!q.pop(b, 10000, stop));  // at once, not after the 10 s
}

int main() {
  fallback_restarts();
  fallback_restarts_spread_out();
  fallback_only_from_40();
  fallback_lost_blocks();
  fallback_lost_blocks_slowly();
  fallback_poll_cadence();
  fallback_cleared();
  fallback_disabled_by_a_wedge_at_20();
  late_recent_counts_the_current_watch();
  late_recent_leaves_out_a_streams_first_moments();
  late_recent_goes_when_the_stream_stops();
  late_recent_starts_over_at_a_fallback();
  gate_hides_the_frames_after_a_gap();
  gate_hides_at_most_so_often();
  gate_switched_off_and_reset();
  queue_in_order();
  queue_full_loses_the_oldest_and_keeps_its_gap();
  queue_buffers_go_round();
  queue_pop_gives_up_when_told_to_stop();
  std::fprintf(stderr, "kp_radio_tests: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
