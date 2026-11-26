// kp_scan_tests: the scan's rules that need no adapter (ctest: "scan"): what it lists (Pilots) and what it watches
// after. Plain checks, as in radio_tests.cpp.
#include <cstdint>
#include <cstdio>
#include <vector>

#include "pilots.h"

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

int main() {
  pilots_seen_again();
  pilots_a_scan_starts_empty();
  pilots_a_copy_is_not_another_pilot();
  pilots_close_together();
  pilots_a_copy_hides_nothing();
  pilots_the_chosen_channel_stays();
  after_scan_the_channel_seen();
  after_scan_the_strongest_else_the_channel();
  after_scan_a_copy();
  std::fprintf(stderr, "kp_scan_tests: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
