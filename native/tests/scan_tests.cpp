// kp_scan_tests: the scan's rules that need no adapter (ctest: "scan"): what it lists (Pilots) and what it watches
// after; its plan and the extra steps it takes (ScanPlan, Steps::insert). Plain checks, as in radio_tests.cpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "pilots.h"
#include "scan.h"
#include "steps.h"

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

// --------------------------------------------------------------------------------------------- Pilots
// A sighting at t ms.
static Pilot seen(double mhz, float snr, int64_t t = 1000) {
  Pilot p;
  p.mhz = mhz;
  p.snr_db = snr;
  p.seen_ms = t;
  return p;
}

// The list's frequencies, strongest first.
static std::vector<double> mhz_of(const Pilots &l) {
  std::vector<double> v;
  for (const auto &p : l.list()) v.push_back(p.mhz);
  return v;
}
using Mhz = std::vector<double>;

// Seen again within kSameMhz: the newer sighting unless it is more than kWeakerDb weaker, which only says the pilot
// is still there. Further away: another pilot. Strongest first.
static void pilots_seen_again() {
  Pilots l;
  CHECK(l.add({seen(5769, 20, 100)}, 0).empty());
  l.add({seen(5770.5, 18, 200), seen(5806, 25, 200)}, 0);
  CHECK(mhz_of(l) == Mhz({5806, 5770.5}) && l.list()[1].snr_db == 18);
  l.add({seen(5769.5, 14.5, 300)}, 0);  // 3.5 dB under: still there, measured as before
  const auto &p = l.list()[1];
  CHECK(l.list().size() == 2 && p.mhz == 5770.5 && p.snr_db == 18 && p.seen_ms == 300);
}

// A scan starts with nothing listed, so the last scan's pilots hide none of this one's.
static void pilots_a_scan_starts_empty() {
  Pilots l;
  l.add({seen(5732, 40)}, 0);
  l.begin_scan();
  CHECK(l.list().empty());
  l.add({seen(5744, 30)}, 0);
  CHECK(mhz_of(l) == Mhz({5744}));
}

// FPVDesktopWiFiReceiver's measurement (README): R3 at 5731.4 MHz, 35 dB, its copy at 5743.4 MHz, 32 dB. One pilot is
// listed, in either order, the two in one step or each in a step of its own.
static void pilots_a_copy_is_not_another_pilot() {
  Pilots l;
  auto c = l.add({seen(5743.4, 32), seen(5731.4, 35)}, 0);
  CHECK(mhz_of(l) == Mhz({5731.4}));
  CHECK(c.size() == 1 && c[0].mhz == 5743.4 && c[0].of_mhz == 5731.4);
  // the VTX first, its copy in a later step: it never shows
  l.begin_scan();
  l.add({seen(5731.4, 35)}, 0);
  c = l.add({seen(5743.4, 32)}, 0);
  CHECK(mhz_of(l) == Mhz({5731.4}) && c.size() == 1);
  // the copy first (below its VTX here): listed until the VTX is seen
  l.begin_scan();
  l.add({seen(5719.4, 32)}, 0);
  CHECK(mhz_of(l) == Mhz({5719.4}));
  l.add({seen(5731.4, 35)}, 0);
  CHECK(mhz_of(l) == Mhz({5731.4}));
  // and seen again later in the scan (the next step overlaps), it goes again
  l.add({seen(5719.6, 31)}, 0);
  CHECK(mhz_of(l) == Mhz({5731.4}));
}

// Two pilots close together: within kCopyDb of each other both stay, as does one kCopyMhz or more away.
static void pilots_close_together() {
  Pilots l;
  l.add({seen(5732, 30), seen(5740, 28.5)}, 0);
  CHECK(mhz_of(l) == Mhz({5732, 5740}));
  l.begin_scan();
  l.add({seen(5732, 30), seen(5747, 20)}, 0);
  CHECK(mhz_of(l) == Mhz({5732, 5747}));
  l.begin_scan();
  l.add({seen(5732, 30), seen(5746.9, 28)}, 0);
  CHECK(mhz_of(l) == Mhz({5732}));
}

// A copy does not take a third pilot with it: one kCopyMhz from the VTX but close to its copy stays.
static void pilots_a_copy_hides_nothing() {
  Pilots l;
  l.add({seen(5732, 40), seen(5744, 37), seen(5756, 34)}, 0);
  CHECK(mhz_of(l) == Mhz({5732, 5756}));
}

// The channel the user is on, or chose, stays, copy or not (within kSameMhz of it); the VTX stays beside it.
static void pilots_the_chosen_channel_stays() {
  Pilots l;
  CHECK(l.add({seen(5731.4, 35), seen(5743.4, 32)}, 5745).empty());
  CHECK(mhz_of(l) == Mhz({5731.4, 5743.4}));
  // chosen before the copy was seen: it stays when it comes
  l.begin_scan();
  l.add({seen(5731.4, 35)}, 5743.4);
  l.add({seen(5743.4, 32)}, 5743.4);
  CHECK(mhz_of(l) == Mhz({5731.4, 5743.4}));
  // another channel chosen: the copy goes
  l.begin_scan();
  l.add({seen(5731.4, 35), seen(5743.4, 32)}, 5748);
  CHECK(mhz_of(l) == Mhz({5731.4}));
}

// A copy kept because it is on the chosen channel hides nothing either: what else is listed is what would be with no
// channel chosen. A third pilot near the copy but kCopyMhz or more from the VTX stays, in the copy's step or a later
// one.
static void pilots_a_chosen_copy_hides_nothing() {
  Pilots l;
  l.add({seen(5731.4, 35), seen(5743.4, 32), seen(5755, 29)}, 0);
  CHECK(mhz_of(l) == Mhz({5731.4, 5755}));
  l.begin_scan();
  CHECK(l.add({seen(5731.4, 35), seen(5743.4, 32), seen(5755, 29)}, 5743.4).empty());
  CHECK(mhz_of(l) == Mhz({5731.4, 5743.4, 5755}));
  l.begin_scan();
  l.add({seen(5731.4, 35), seen(5743.4, 32)}, 5743.4);
  CHECK(l.add({seen(5755, 29)}, 5743.4).empty());
  CHECK(mhz_of(l) == Mhz({5731.4, 5743.4, 5755}));
}

// ----------------------------------------------------------------------------------------- after the scan
// The channel watched before (or asked for) if the scan saw it, within kSameMhz, however weak; of several, the last in
// the list (the weakest), not the nearest.
static void after_scan_the_channel_seen() {
  Pilots l;
  l.add({seen(5806, 35), seen(5770.3, 15)}, 0);
  CHECK(l.watch_after_scan(5769) == 5770.3);
  CHECK(l.watch_after_scan(5806.5) == 5806);
  CHECK(l.watch_after_scan(5735.9) == 5806);  // not seen: the strongest
  l.begin_scan();
  l.add({seen(5806, 35), seen(5766, 16), seen(5770.3, 15)}, 0);
  CHECK(l.watch_after_scan(5767) == 5770.3);
  l.begin_scan();
  l.add({seen(5806, 35), seen(5766, 15), seen(5770.3, 16)}, 0);
  CHECK(l.watch_after_scan(5769) == 5766);
}

// Else the strongest this scan saw; with nothing seen, the channel all the same; with no channel either, nothing.
static void after_scan_the_strongest_else_the_channel() {
  Pilots l;
  CHECK(l.watch_after_scan(0) == 0);
  CHECK(l.watch_after_scan(5769) == 5769);
  l.add({seen(5732, 20), seen(5806, 30)}, 0);
  CHECK(l.watch_after_scan(0) == 5806);
  CHECK(l.watch_after_scan(5735.9) == 5732);
  CHECK(l.watch_after_scan(5736.1) == 5806);  // just over kSameMhz away
  // the last scan's pilots are gone: the one asked for is not there, and this scan's strongest is
  l.begin_scan();
  l.add({seen(5769, 10)}, 0);
  CHECK(l.watch_after_scan(5806) == 5769);
  l.begin_scan();
  CHECK(l.watch_after_scan(5806) == 5806);
}

// A copy is never the strongest. On the user's channel it is kept, and watched: the scan does not move them off it.
static void after_scan_a_copy() {
  Pilots l;
  l.add({seen(5743.4, 32), seen(5731.4, 35)}, 0);
  CHECK(l.watch_after_scan(0) == 5731.4);
  l.begin_scan();
  l.add({seen(5743.4, 32), seen(5731.4, 35)}, 5743.4);
  CHECK(l.watch_after_scan(5743.4) == 5743.4);
}

// ------------------------------------------------------------------------------- the plan and its extra steps
// A signal on the air from lo to hi MHz, snr_db over the floor.
struct Air {
  double lo, hi;
  float snr_db = 30;
};

// The hardware centre the chip takes for a step (Hz).
static double grid(double hz) { return std::round(hz / ScanPlan::kGrid) * ScanPlan::kGrid; }

// What find_step_signals makes of the air in a step with the hardware centre c (Hz): the humps whole inside +-kEdge,
// kHumpMinHz to kHumpMaxHz wide, and the signals cut by one edge, at least kHumpMinHz of each inside. One past both
// edges leaves nothing over the floor.
static StepSignals look(double c, const std::vector<Air> &air) {
  constexpr double kEdge = ScanPlan::kEdge;
  StepSignals s;
  for (const auto &a : air) {
    const double f0 = a.lo * 1e6 - c, f1 = a.hi * 1e6 - c;
    const bool lo = f0 <= -kEdge, hi = f1 >= kEdge;
    if (!lo && !hi && f1 - f0 >= kHumpMinHz && f1 - f0 <= kHumpMaxHz) {
      s.whole.push_back(Hump{(f0 + f1) / 2, f1 - f0, a.snr_db, a.snr_db});
    } else if (lo != hi) {
      CutSignal cut;
      cut.f0_hz = std::max(f0, -kEdge);
      cut.f1_hz = std::min(f1, kEdge);
      cut.cut_hi = hi;
      cut.snr_db = a.snr_db;
      if (cut.f1_hz - cut.f0_hz >= kHumpMinHz) s.cut.push_back(cut);
    }
  }
  return s;
}

// A whole scan as the radio takes one (radio.cpp: handle_scan): the plan through Steps, each step on the chip's grid,
// the extra steps ScanPlan gives inserted after the step under way. What it did: the hardware centres in order (MHz),
// the middles of the signals seen whole (MHz), the extra steps.
struct Scan {
  std::vector<double> steps, whole;
  std::vector<ScanPlan::Extra> extra;
};
static Scan scan_over(const std::vector<Air> &air) {
  ScanPlan plan;
  Steps s;
  s.start(plan.start(), 1);
  Scan out;
  do {
    const double c = grid(s.hz());
    const StepSignals seen = look(c, air);
    out.steps.push_back(c / 1e6);
    for (const auto &h : seen.whole) out.whole.push_back((c + h.offset_hz) / 1e6);
    const auto extra = plan.step(c, seen.whole, seen.cut);
    s.insert(ScanPlan::hz_of(extra));
    out.extra.insert(out.extra.end(), extra.begin(), extra.end());
  } while (s.next());
  CHECK(out.extra.size() == plan.given() && out.steps.size() == ScanPlan::plan().size() + plan.given());
  return out;
}

static bool seen_whole(const Scan &sc, double mhz) {
  for (double w : sc.whole)
    if (std::fabs(w - mhz) < 0.5) return true;
  return false;
}

// The step taken right after the one at mhz (0: none).
static double after(const Scan &sc, double mhz) {
  for (size_t i = 0; i + 1 < sc.steps.size(); i++)
    if (sc.steps[i] == mhz) return sc.steps[i + 1];
  return 0;
}

// A scan whose next step is the plan's step at mhz (on the chip's grid), the plan's steps before it empty.
static ScanPlan scan_at(double mhz) {
  ScanPlan p;
  for (double hz : p.start()) {
    if (grid(hz) == mhz * 1e6) break;
    p.step(grid(hz), {}, {});
  }
  return p;
}

static CutSignal cut_part(double f0_mhz, double f1_mhz, bool cut_hi, float snr_db = 30) {
  CutSignal c;
  c.f0_hz = f0_mhz * 1e6;
  c.f1_hz = f1_mhz * 1e6;
  c.cut_hi = cut_hi;
  c.snr_db = snr_db;
  return c;
}

// The plan: the upper bands from 5652 MHz, then the low band from 5370, kStep apart. A new scan starts it over, with
// nothing of the last scan left: no step owed, none given, nothing waiting.
static void the_plan_and_a_new_scan() {
  const auto p = ScanPlan::plan();
  CHECK(p.size() == 29 && p.front() == 5652e6 && p[14] == 5932e6 && p[15] == 5370e6 && p.back() == 5630e6);
  for (size_t i = 1; i < p.size(); i++)
    if (i != 15) CHECK(p[i] - p[i - 1] == ScanPlan::kStep);
  ScanPlan s;
  CHECK(s.start() == p);
  for (int scan = 0; scan < 2; scan++) {
    CHECK(s.step(5650e6, {}, {cut_part(-16.5, -10, false)}).empty());
    CHECK(s.step(5670e6, {}, {cut_part(10, 16.5, true)}).size() == 1 && s.given() == 1);
    CHECK(s.start() == p);
  }
}

// F2 (5751-5769 MHz) halfway between two steps: the 5750 one sees its bottom end, the 5770 one its top end, neither
// sees it whole. One step centred on it, right after the 5770 one, sees it whole.
static void extra_step_centred_on_a_signal_cut_by_two() {
  const Scan sc = scan_over({{5751, 5769}});
  CHECK(after(sc, 5750) == 5770 && after(sc, 5770) == 5760 && after(sc, 5760) == 5790);
  CHECK(sc.extra.size() == 1);
  if (sc.extra.size() == 1)
    CHECK(sc.extra[0].hz == 5760e6 && sc.extra[0].lo_hz == 5751e6 && sc.extra[0].hi_hz == 5769e6);
  CHECK(seen_whole(sc, 5760));
  // one kHumpMaxHz wide still fits (F2 and A7, 5 MHz off a step)
  CHECK(seen_whole(scan_over({{5748, 5772}}), 5760));
  CHECK(seen_whole(scan_over({{5733, 5757}}), 5745));
}

// A signal cut by one step that the next sees whole (R4 at 5760-5778, cut by the 5750 step's top edge, whole in the
// 5770 one) gets no step, nor does one seen whole before a step cuts it (R3 at 5723-5741: whole in the 5730 step,
// its top cut by the 5750 one's bottom edge).
static void no_extra_step_for_a_signal_seen_whole() {
  Scan sc = scan_over({{5760, 5778}});
  CHECK(sc.extra.empty() && seen_whole(sc, 5769));
  sc = scan_over({{5723, 5741}});
  CHECK(sc.extra.empty() && seen_whole(sc, 5732));
}

// Cut by the last step of a band, a signal gets its step at once, kReach inside the end seen: E8 (5936-5954), cut by
// the top edge of the highest step (5930), before the plan goes on to the low band; and at the plan's last step. One
// cut by the first step of a band, which the next does not see at all, gets its step after that one: L1 at 19 MHz
// wide (5352.5-5371.5).
static void extra_step_from_the_one_end_seen() {
  Scan sc = scan_over({{5936, 5954}});
  CHECK(after(sc, 5930) == 5950 && after(sc, 5950) == 5370);
  CHECK(sc.extra.size() == 1);
  if (sc.extra.size() == 1) CHECK(sc.extra[0].hz == 5948e6 && sc.extra[0].lo_hz == 5936e6 && sc.extra[0].hi_hz == 0);
  CHECK(seen_whole(sc, 5945));
  ScanPlan last = scan_at(5630);
  const auto e = last.step(5630e6, {}, {cut_part(10, 16.5, true)});
  CHECK(e.size() == 1 && e[0].hz == 5652e6);
  sc = scan_over({{5352.5, 5371.5}});
  CHECK(after(sc, 5370) == 5390 && after(sc, 5390) == 5360 && after(sc, 5360) == 5410);
  CHECK(sc.extra.size() == 1);
  if (sc.extra.size() == 1) CHECK(sc.extra[0].hi_hz == 5371.5e6 && sc.extra[0].lo_hz == 0);
  CHECK(seen_whole(sc, 5362));
}

// A signal inside a step already given gets no other, even one that step did not see whole: 5755-5778 MHz, its
// bottom end cut by the 5750 step's top edge, off the air while the 5770 step and the step given (5767) are taken,
// then its top end seen by the 5790 step's bottom edge.
static void no_extra_step_inside_one_given() {
  ScanPlan s = scan_at(5750);
  CHECK(s.step(5750e6, {}, {cut_part(5, 16.5, true)}).empty());  // waits for the next step
  const auto e = s.step(5770e6, {}, {});                         // which does not see it
  CHECK(e.size() == 1 && e[0].hz == 5767e6 && e[0].lo_hz == 5755e6);
  CHECK(s.step(5765e6, {}, {}).empty());  // the step given, which does not see it either
  CHECK(s.step(5790e6, {}, {cut_part(-16.5, -12, false)}).empty());
  CHECK(s.step(5810e6, {}, {}).empty() && s.given() == 1);
}

// At most kMax extra steps a scan: with a VTX halfway between every two steps of the upper bands (15 of them, F1-F8
// and R7 among them), the first kMax get one each, in the order the scan meets them.
static void at_most_kmax_extra_steps() {
  std::vector<Air> air;
  for (double mhz = 5660; mhz <= 5940; mhz += 20) air.push_back({mhz - 9, mhz + 9});
  const Scan sc = scan_over(air);
  CHECK(sc.extra.size() == ScanPlan::kMax);
  for (size_t i = 0; i < air.size(); i++) CHECK(seen_whole(sc, 5660 + 20.0 * i) == (i < ScanPlan::kMax));
}

// What an extra step sees cut asks for nothing, and an extra step does not stand in for the next step of the plan:
// 5731-5749 MHz, cut by the 5730 and 5750 steps, gets a step at 5740, which sees the top of F2 (5751-5769) cut; F2,
// cut by the 5750 step too, still waits for the 5770 one.
static void extra_steps_ask_for_nothing() {
  const Scan sc = scan_over({{5731, 5749}, {5751, 5769}});
  CHECK(after(sc, 5750) == 5740 && after(sc, 5740) == 5770 && after(sc, 5770) == 5760);
  CHECK(sc.extra.size() == 2 && seen_whole(sc, 5740) && seen_whole(sc, 5760));
}

// A cut signal under kMinSnrDb gets no step (a wide Wi-Fi's skirts, noise); a weak part still counts as one end of
// a signal whose other part is strong enough.
static void no_extra_step_for_a_weak_signal() {
  CHECK(scan_over({{5751, 5769, 8}}).extra.empty());
  CHECK(scan_over({{5936, 5954, 9}}).extra.empty());
  ScanPlan s = scan_at(5750);
  CHECK(s.step(5750e6, {}, {cut_part(1, 16.5, true, 8)}).empty());
  const auto e = s.step(5770e6, {}, {cut_part(-16.5, -1, false, 14)});
  CHECK(e.size() == 1 && e[0].hz == 5760e6 && e[0].snr_db == 14);
}

// Both ends of a signal over kHumpMaxHz wide seen (no step would take it whole): no step. One over 23 MHz wide can be
// cut by the same edge of two steps in a row (5760-5790 MHz: its bottom end by the top edge of the 5750 and of the
// 5770 step); it waits for its other end all the same.
static void no_extra_step_for_a_signal_too_wide() {
  CHECK(scan_over({{5745, 5775}}).extra.empty());
  CHECK(scan_over({{5760, 5790}}).extra.empty());
}

// Every channel of every band is seen whole in some step, the plan's or one extra step, at any width up to the
// kHumpMaxHz find_step_signals takes. One under 13 MHz wide needs no extra step, but for E8 (5945 MHz), past the top
// edge of the highest step (5930).
static void the_scan_sees_every_channel_whole() {
  for (double width : {6.6, 12.5, 15.0, 17.0, 19.0, 21.0, 24.0})
    for (const auto &ch : fpv_channels()) {
      const Scan sc = scan_over({{ch.mhz - width / 2, ch.mhz + width / 2}});
      const size_t most = width < 13 && std::string(ch.name) != "E8" ? 0 : 1;
      const bool ok = seen_whole(sc, ch.mhz) && sc.extra.size() <= most;
      if (!ok) std::fprintf(stderr, "%s at %.1f MHz wide: %zu extra steps\n", ch.name, width, sc.extra.size());
      CHECK(ok);
    }
}

// Extra steps inserted come right after the one under way, in their order, before the rest of the plan; size()
// counts them. At the plan's last step they go on its end. A new plan has none.
static void steps_take_inserted_steps_next() {
  const auto bank = std::vector<cf>(100, cf(1, 0));
  Steps s;
  s.start({5650e6, 5670e6}, 100);
  s.insert({5660e6, 5661e6});
  CHECK(s.size() == 4 && s.at() == 0 && s.hz() == 5650e6);
  CHECK(s.next() && s.hz() == 5660e6);
  bool whole = false;  // captured like any other step
  for (uint64_t seq = 0; seq <= Steps::kSettleBanks; seq++) whole = s.take(bank, seq, 1, 5660e6);
  CHECK(whole && s.iq().size() == 100 && s.centre_hz() == 5660e6);
  CHECK(s.next() && s.hz() == 5661e6);
  CHECK(s.next() && s.hz() == 5670e6 && s.at() == 3);
  s.insert({5680e6});
  CHECK(s.size() == 5 && s.next() && s.hz() == 5680e6);
  CHECK(!s.next() && s.done());
  s.start({5700e6}, 100);
  CHECK(s.size() == 1);
}

int main() {
  pilots_seen_again();
  pilots_a_scan_starts_empty();
  pilots_a_copy_is_not_another_pilot();
  pilots_close_together();
  pilots_a_copy_hides_nothing();
  pilots_the_chosen_channel_stays();
  pilots_a_chosen_copy_hides_nothing();
  after_scan_the_channel_seen();
  after_scan_the_strongest_else_the_channel();
  after_scan_a_copy();
  the_plan_and_a_new_scan();
  extra_step_centred_on_a_signal_cut_by_two();
  no_extra_step_for_a_signal_seen_whole();
  extra_step_from_the_one_end_seen();
  no_extra_step_inside_one_given();
  at_most_kmax_extra_steps();
  extra_steps_ask_for_nothing();
  no_extra_step_for_a_weak_signal();
  no_extra_step_for_a_signal_too_wide();
  the_scan_sees_every_channel_whole();
  steps_take_inserted_steps_next();
  std::fprintf(stderr, "kp_scan_tests: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
