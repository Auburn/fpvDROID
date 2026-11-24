// fpvDROID radio: whether a watch at 40 MS/s is holding up, and what follows when it is not.
#pragma once
#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>

namespace kp {

// A watch at 40 MS/s whose stream does not hold up goes on at 20 MS/s (the same picture: the BW20 RX filter is
// +-10 MHz either way) until the radio is opened again or the watch rate is set anew. Two kinds of evidence:
//  - restarts: kFaults of them within kWindowMs. A link that cannot carry 40 MS/s restarts every 0.2-0.3 s
//    (Chrome's WebUSB: ~14k transfers/s against the 24.4k needed); a healthy one runs minutes without one.
//  - lost blocks: banks the stream dropped without restarting (collected late or overrun: wifi-sdr
//    RxStatus::mid_drops), kDrops of them within kWindowMs. At 40 MS/s a bank is 205 us, ~4900 of them a second,
//    and each one lost takes 3 lines out of the picture. A host that stalls now and then loses one to three a
//    minute, at most four at one go: left alone. kDrops in the window is two a second kept up.
// Neither counts in the first kGraceMs of a watch. On some hosts a fresh stream starts badly for a while: a
// Pixel 8 Pro loses the run at bank 51-55 for one to four seconds, a restart each time, before it keeps up for
// good. A link that cannot carry 40 MS/s goes on restarting past the grace, and still falls back.
// On that phone 20 MS/s is the better rate, now that the stream frees its USB path's wedge without stopping (the
// wifi-sdr patch, patches/README.md; before, 20 MS/s wedged within ~0.2 s there and was far the worse). Measured:
// one restart in ~15 minutes at 20 MS/s, two to four a minute at 40 MS/s. And what cost a restart at 40 MS/s
// there now mostly costs one to four lost blocks instead, so it is the lost blocks that can take such a watch to
// 20 MS/s (in one run of three, 20 s in), where the restarts seldom did.
// And none at all once a stream that had fallen back wedged at 20 MS/s for good (the receiver stopped: the wedge
// was back straight after being freed): on such a host 20 MS/s is worse than a 40 MS/s that restarts, so there is
// no more falling back for as long as this object lives (clear() keeps that).
// A third kind of evidence needs no grace: a 40 MS/s watch whose receiver gives up altogether (its stream failed
// five starts in a row without a bank). The app opens the adapter again, and the fallback used to be cleared by
// that and to come too late for the grace, so it went round for ever: an LG G7+, whose 40 MS/s runs all end
// within ten banks, opened the adapter 13 times in 2.5 minutes. Such a stop falls back at once, and that
// fallback holds over the reopens until the watch rate is set anew (clear(true)). Not after a wedge, nor once the
// falling back has been given up.
//
// The lost blocks are counted at every sample rate, for late_recent(): how many the current watch has lost
// lately, which is what tells the app that the host is not keeping up. Not the ones a stream loses in its first
// kSettleMs: on some hosts every fresh stream loses a few blocks about 55 blocks in (a Pixel 8 Pro: three or four,
// 11 ms after its first), which says nothing of how the host keeps up from there on.
//
// The clock is the caller's: every call that needs the time is given it, in ms of one steady clock.
// Thread-safe: the DSP thread feeds it, the status and the setters read and clear it from other threads.
class FallbackPolicy {
public:
  static constexpr int kFaults = 3;
  static constexpr int kDrops = 10;
  static constexpr int64_t kWindowMs = 5000;
  static constexpr int64_t kGraceMs = 10000;
  static constexpr int64_t kPollMs = 200;  // how often the receiver's count of lost blocks is wanted
  static constexpr int64_t kSettleMs = 250;

  // ---- any thread
  bool active() const;    // watches asked for at 40 MS/s run at 20 MS/s
  bool disabled() const;  // 20 MS/s wedged after a fallback: no more falling back
  // The rate_sel a watch runs at, asked for at `asked` (1 = 40 MS/s, 2 = 20 MS/s).
  int rate_for(int asked) const;
  // Watches run at the rate asked for again: the radio opened anew, or (rate_set) the watch rate set anew. A
  // fallback from a 40 MS/s stream that never got going (receiver_failed) holds over the first, not the second.
  void clear(bool rate_set = false);
  // Blocks the current watch's stream lost in the last kWindowMs, as of the last poll: counted from kSettleMs
  // after that watch's first block (nothing from before its retune, nor the stream's first moments), and again
  // so after the retune of a fallback. 0 outside a watch, and once the polls stop for kWindowMs (a stream that
  // delivers nothing).
  uint64_t late_recent(int64_t now_ms) const;

  // ---- the DSP thread
  void watch_started(int64_t now_ms);  // a new watch: nothing counted yet, the grace runs from now
  void watch_ended();                  // the radio left the watch for another mode
  // The stream restarted during a watch at fs. True: fall back now (active() from here on).
  bool stream_restarted(int64_t now_ms, double fs);
  // The receiver's count of lost blocks is due: read it and call blocks_lost(). Asked with every bank of a watch.
  bool poll_due(int64_t now_ms);
  // lost_total: the receiver's count, which only ever rises. True: fall back now (active() from here on).
  bool blocks_lost(int64_t now_ms, double fs, uint64_t lost_total);
  // The receiver stopped with its C2H path wedged; watching: the radio was in a watch.
  void receiver_wedged(bool watching);
  // The receiver stopped because its stream kept failing (not wedged, not unplugged), in a watch (watching) at fs.
  // True: it fell back now (active() from here on, over reopens).
  bool receiver_failed(bool watching, double fs);

private:
  bool counts(int64_t now_ms, double fs) const;
  void fall_back();

  mutable std::mutex _m;
  bool _active = false, _disabled = false, _watching = false;
  bool _held = false;                                 // the fallback came from receiver_failed: open() keeps it
  int64_t _t0_ms = 0;                                 // when the watch began
  int64_t _next_poll_ms = 0;
  int64_t _first_ms = -1;                             // the first poll of this watch's stream (-1: none yet)
  std::deque<int64_t> _restarts;                      // the restarts that count, the last kWindowMs of them
  std::deque<std::pair<int64_t, uint64_t>> _marks;    // (ms, lost_total) at each poll, the last kWindowMs of them
};

}  // namespace kp
