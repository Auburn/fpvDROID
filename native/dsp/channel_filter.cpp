// fpvDROID front end: the channel filter. See channel_filter.h.
#include "channel_filter.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace kp {

ChannelFilter::ChannelFilter(double fs) : _fs(fs), _hist_r(kHistory, 0.f), _hist_q(kHistory, 0.f) {}

void ChannelFilter::steer(const Sync &s, double seconds) {
  Want want = _want.load(std::memory_order_relaxed);
  const float width = want.width;
  if (s.snr <= 0) {
    // No lock, or no levels yet. The filter stays as it is, which a weak signal needs to be found again; but not
    // for long, in case it is the filter that is in the way.
    if (s.acquiring && width > 0 && (_lost += seconds) > kLostS) {
      want.width = 0;
      _want.store(want, std::memory_order_relaxed);
      _at_n = 0;
      _noise.reset();
    }
    return;
  }
  _lost = 0;
  // Where the middle of the signal's swing is: 1.75 sync depths above the tip, with the tip and the depth as the
  // decoder has them now. A weak signal's levels are not the transmitter's: they are pulled towards the middle of
  // whatever band reaches the discriminator, which is the filter's own middle once it is in, and the depth
  // shrinks by as much. Tip and depth as they are measured still lead there: each time the filter is put where
  // they say, it is that much nearer (by the part of the swing that is left), and it never runs past. (With a
  // depth from elsewhere under a shrunk tip it runs away: every step pulls the tip further.) Smoothed over
  // kSmoothS, so that levels that jump with the picture do not have the filter chasing them.
  const float depth = s.blank - s.tip, at = s.tip + 1.75f * depth;
  const float sm = _at_n++ == 0 ? 1.f : static_cast<float>(std::min(1.0, seconds / kSmoothS));
  _at += sm * (at - _at);
  _now += sm * (depth - _now);
  // How wide: by the transmitter's sync depth, which is the deepest seen (a weak signal's only ever shrinks; the
  // smoothed one, or the noise in a weak signal's levels would make it deeper and deeper)
  _depth = std::max({_depth * static_cast<float>(1 - seconds / 60), _now, s.depth_ref});
  // In or out: by how much of the last tenth of a second or so the sync was noisy (NoisyLatch; noisy: the filter
  // is in).
  if (!_noise.update(s.snr, static_cast<float>(std::min(1.0, seconds / kJudgeS)))) {
    if (width > 0) {
      want.width = 0;
      _want.store(want, std::memory_order_relaxed);
    }
    return;
  }
  // (no depth worth the name seen yet: as wide as for the transmitter that swings widest, which costs the others
  // some of the gain and breaks no picture)
  const float w = _depth >= kMinDepth ? std::clamp(std::round(kDepths * _depth), kMinMhz, kMaxMhz) : kMaxMhz;
  if (width <= 0 || std::fabs(w - width) >= 2) want.width = w;
  if (width <= 0 || std::fabs(_at - want.centre) > 0.5f) want.centre = std::round(_at * 4) / 4;
  _want.store(want, std::memory_order_relaxed);
}

bool ChannelFilter::retune() {
  const Want want = _want.load(std::memory_order_relaxed);
  const float w = want.width, c = w > 0 ? want.centre : 0;
  if (w == _width && c == _centre) return false;
  _width = w;
  _centre = c;
  _changes.fetch_add(1, std::memory_order_relaxed);
  if (w > 0) {  // a windowed-sinc low-pass
    const double fc = 0.5e6 * w / _fs;  // its edge, of the sample rate
    double sum = 0;
    for (int i = 0; i < kTaps; i++) {
      const double k = i - (kTaps - 1) / 2.0;
      const double win = 0.42 - 0.5 * std::cos(2 * M_PI * i / (kTaps - 1)) + 0.08 * std::cos(4 * M_PI * i / (kTaps - 1));
      _h[i] = static_cast<float>((k == 0 ? 2 * fc : std::sin(2 * M_PI * fc * k) / (M_PI * k)) * win);
      sum += _h[i];
    }
    for (float &h : _h) h = static_cast<float>(h / sum);
  }
  return true;
}

ChannelFilter::Stream ChannelFilter::run(float *r, float *q, int n) {
  std::memcpy(r, _hist_r.data(), kHistory * sizeof(float));
  std::memcpy(q, _hist_q.data(), kHistory * sizeof(float));
  std::memcpy(_hist_r.data(), r + n, kHistory * sizeof(float));
  std::memcpy(_hist_q.data(), q + n, kHistory * sizeof(float));
  if (_width <= 0) return {r + kHistory / 2, q + kHistory / 2};
  fir(r, n, _fr);
  fir(q, n, _fq);
  return {_fr.data(), _fq.data()};
}

// One half of the stream (I or Q) through the filter: n samples out of the n + kHistory at s.
void ChannelFilter::fir(const float *s_in, int n, std::vector<float> &out_v) const {
  constexpr int Hc = kHistory, D = Hc / 2;
  out_v.resize(static_cast<size_t>(n));
  const float *__restrict s = s_in;
  float *__restrict out = out_v.data();
  const float *h = _h;
  for (int j = 0; j < n; j++) {  // (the taps are the same from either end)
    float acc = h[D] * s[j + D];
    for (int t = 0; t < D; t++) acc += h[t] * (s[j + t] + s[j + Hc - t]);
    out[j] = acc;
  }
}

}  // namespace kp
