// fpvDROID front end: the channel filter (DecoderOpts::channel_filter). The stream has no filter of its own
// narrower than its sample rate: at 20 MS/s close to 20 MHz of noise reach the discriminator, and an FM picture
// needs about 10 dB of carrier over that noise before it is free of sparkles (below that it loses contrast as
// well). The signal itself is narrower. So while the decoder finds the sync noisy (its depth less than kOn times
// the noise on the back porch; until it is more than kOff times with the filter in: sync_noise.h), the stream goes
// through a low-pass kDepths sync depths wide around the middle of the signal's swing, sync tip to white, as the
// decoder has them from better times. That width is what recordings of three transmitters with noise added came
// out best with, each against its own pictures without the noise: +0.3 to +1.3 dB of picture with the carrier
// 12 dB over the noise, +2 to +4.5 dB at 3 to 6 dB, and four lines in five finding their hsync where half did.
// Narrower than the swing and the picture breaks up; on a clean signal the filter costs fine detail, so it is out
// then. Out, the stream is only delayed by as much as the filter delays it, so that the picture does not step
// sideways when it comes in.
//
// Two sides, as the front end has two (FrontEnd::demod, decode), each on its own thread: decode()'s says what is
// wanted (steer), demod()'s does it (retune, run). What is wanted crosses from the one to the other whole.
#pragma once
#include <atomic>
#include <cstdint>
#include <vector>

#include "sync_noise.h"

namespace kp {

class ChannelFilter {
public:
  static constexpr int kTaps = 23;
  static constexpr int kHistory = kTaps - 1;  // samples of the stream that run() wants room for ahead of a chunk

  // fs: the stream's sample rate where the filter sits (just ahead of the discriminator)
  explicit ChannelFilter(double fs);

  // ---- decode()'s side ----
  // The sync as the decoder has it.
  struct Sync {
    bool acquiring;    // there is no lock
    float snr;         // the sync depth over the noise on the back porch (0: not known)
    float tip, blank;  // the levels, MHz of discriminator output
    float depth_ref;   // the sync depth of better times (MHz; 0: none yet)
  };
  // After every block (`seconds` of signal): in or out, by how noisy the decoder finds the sync, and how wide and
  // where, by the levels it has. In whole MHz and quarters of one, so that it is not retuned for nothing.
  void steer(const Sync &s, double seconds);

  // ---- demod()'s side ----
  // The filter as steer() last wanted it. True: it has changed (the caller tunes its mixer again: centre()).
  bool retune();
  // The filter's middle, MHz above the carrier (0 with the filter out). The mixer brings it to 0, a low-pass does
  // the rest, and the discriminator's output gets it added back, so that the decoder's levels are where they were.
  float centre() const { return _centre; }
  struct Stream {
    const float *r, *q;
  };
  // A chunk of the stream, n samples of I at r[kHistory] on and of Q at q[kHistory] on (the kHistory entries in
  // front are this one's to fill: the stream's last samples before the chunk). What the discriminator reads, n
  // samples each: the chunk through the filter, or with the filter out half its length early: delayed by as much
  // as the filter delays, without a sample being moved.
  Stream run(float *r, float *q, int n);

  // ---- any thread ----
  float width() const { return _want.load(std::memory_order_relaxed).width; }  // as wanted, MHz (0: out)
  uint64_t changes() const { return _changes.load(std::memory_order_relaxed); }  // times it came in, went out or was retuned

private:
  static constexpr float kOn = 24, kOff = 36, kDepths = 6, kMinMhz = 7, kMaxMhz = 13;
  static constexpr float kMinDepth = 1.0f;  // MHz: a sync depth below any transmitter's (they are 1.3 to 2.0)
  static constexpr double kJudgeS = 0.1;    // in or out is judged over about this long;
  static constexpr double kLostS = 1.0;     // out when the lock has been gone for this long;
  static constexpr double kSmoothS = 0.3;   // and the filter's middle follows the levels over this long

  void fir(const float *s, int n, std::vector<float> &out) const;

  double _fs;
  // What steer() wants and retune() takes: one value, stored and read whole.
  struct Want {
    float width = 0, centre = 0;  // MHz: how wide (0: out), and its middle above the carrier
  };
  std::atomic<Want> _want{};
  std::atomic<uint64_t> _changes{0};
  // steer(): is the sync noisy (noisy: the filter is in); how long the lock has been gone; where the levels put
  // the filter's middle and the sync depth, both smoothed (_at_n: times measured); and the deepest sync depth seen
  NoisyLatch _noise{kOn, kOff};
  double _lost = 0;
  float _at = 0, _now = 0, _depth = 0;
  int _at_n = 0;
  // retune(), run(): the filter in use, its taps, the stream's last kHistory samples (I and Q), and a chunk
  // through the filter
  float _width = 0, _centre = 0;
  float _h[kTaps] = {};
  std::vector<float> _hist_r, _hist_q;
  std::vector<float> _fr, _fq;
};

}  // namespace kp
