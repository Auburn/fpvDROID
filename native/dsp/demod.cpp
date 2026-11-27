// fpvDROID front end, demod()'s half: IQ at the stream's rate in, discriminator output (MHz) out. See video.h.
// fpvDROID: block-wise and branch-free so the compiler vectorises it (the spike's per-sample loop used a whole
// phone core at 40 MS/s). Per chunk of <= kTab samples: DC removal (block mean, smoothed), mix by a
// precomputed rotation table, halfband /2 over a work array with 22 samples of history, the channel filter while
// the sync is noisy (channel_filter.h), discriminator.
#include <algorithm>
#include <cmath>
#include <cstring>

#include "channel_filter.h"
#include "sparkle.h"
#include "video.h"

namespace kp {

void FrontEnd::set_shift(double s) {
  _shift = s;
  tune(_shift + (_chan ? _chan->centre() : 0.f) * 1e6);
}

// The mixer brings hz to 0.
void FrontEnd::tune(double hz) {
  _tr.resize(kTab);
  _ti.resize(kTab);
  for (int i = 0; i < kTab; i++) {
    const double a = -2 * M_PI * hz / _fs_in * i;
    _tr[i] = static_cast<float>(std::cos(a));
    _ti[i] = static_cast<float>(std::sin(a));
  }
  const double a = -2 * M_PI * hz / _fs_in * kTab;
  _stepN = cf(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
  _rot = cf(1, 0);
}

namespace {
// branch-free atan2 (a polynomial, max error ~1e-4 rad) so the discriminator loop vectorises
inline float atan2_bf(float y, float x) {
  const float ax = std::fabs(x), ay = std::fabs(y);
  const float mx = ax > ay ? ax : ay, mn = ax > ay ? ay : ax;
  const float a = mn / (mx + 1e-20f);
  const float s = a * a;
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  r = ay > ax ? 1.57079637f - r : r;
  r = x < 0 ? 3.14159274f - r : r;
  return y < 0 ? -r : r;
}
}  // namespace

void FrontEnd::demod(const cf *xs, size_t n, std::vector<float> &out, std::vector<uint8_t> &bad) {
  out.clear();
  // the channel filter as decode() last wanted it: its middle goes to 0 (the mixer), and comes back onto the
  // discriminator's output below
  if (_chan && _chan->retune()) tune(_shift + _chan->centre() * 1e6);
  const float centre = _chan ? _chan->centre() : 0.f;
  const bool rep = _picture.load(std::memory_order_relaxed).repair;
  _clicks->begin(rep);
  bad.clear();
  const float k = static_cast<float>(_fs_out / (2 * M_PI) / 1e6);  // rad/sample -> MHz
  constexpr int H = kHb - 1;
  const int pad = _chan ? ChannelFilter::kHistory : 0;  // room for its history, in front of each chunk's samples
  while (n > 0) {
    const int m = static_cast<int>(std::min<size_t>(n, kTab));
    // DC: block mean, smoothed across blocks (the adapter's DC line)
    float sr = 0, si = 0;
    for (int i = 0; i < m; i++) {
      sr += xs[i].real();
      si += xs[i].imag();
    }
    _dc += (cf(sr / m, si / m) - _dc) * 0.3f;
    const float dr = _dc.real(), di = _dc.imag();
    const float pr = _rot.real(), pi = _rot.imag();
    float *wr = _wr.data() + H, *wi = _wi.data() + H;
    const float *tr = _tr.data(), *ti = _ti.data();
    for (int i = 0; i < m; i++) {
      const float xr = xs[i].real() - dr, xi = xs[i].imag() - di;
      const float cr = pr * tr[i] - pi * ti[i], ci = pr * ti[i] + pi * tr[i];  // rot * step^i
      wr[i] = xr * cr - xi * ci;
      wi[i] = xr * ci + xi * cr;
    }
    _rot *= _stepN;
    if (m < kTab) {  // a short chunk: the next one starts at step^m, not step^kTab
      _rot = cf(pr, pi) * cf(tr[m], ti[m]);
    }
    _rot /= std::abs(_rot);
    // halfband /2 (or pass through), then the discriminator
    const size_t o0 = out.size();
    if (_decimate) {
      const float *r = _wr.data(), *q = _wi.data();
      int j0 = _phase;
      const int nout = m - 1 - j0 >= 0 ? (m - 1 - j0) / 2 + 1 : 0;  // windows that end inside this chunk
      _yr.resize(static_cast<size_t>(pad + nout));
      _yi.resize(static_cast<size_t>(pad + nout));
      float *yr_w = _yr.data() + pad, *yi_w = _yi.data() + pad;
      for (int j = 0; j < nout; j++) {
        const int s0 = j0 + 2 * j;
        float a = 0.5f * r[s0 + kHb / 2], b = 0.5f * q[s0 + kHb / 2];
        for (int t = 0; t < kHb; t += 2) {
          a += _h[t] * r[s0 + t];
          b += _h[t] * q[s0 + t];
        }
        yr_w[j] = a;
        yi_w[j] = b;
      }
      _phase = j0 + 2 * nout - m;  // where the next window starts, relative to the next chunk
      std::memmove(_wr.data(), _wr.data() + m, H * sizeof(float));
      std::memmove(_wi.data(), _wi.data() + m, H * sizeof(float));
    } else {
      _yr.resize(static_cast<size_t>(pad + m));
      _yi.resize(static_cast<size_t>(pad + m));
      std::memcpy(_yr.data() + pad, wr, static_cast<size_t>(m) * sizeof(float));
      std::memcpy(_yi.data() + pad, wi, static_cast<size_t>(m) * sizeof(float));
    }
    const int ny = static_cast<int>(_yr.size()) - pad;
    // What the discriminator reads: the chunk, or with the channel filter what that makes of it
    const float *yr = _yr.data(), *yi = _yi.data();
    if (_chan && ny > 0) {
      const ChannelFilter::Stream f = _chan->run(_yr.data(), _yi.data(), ny);
      yr = f.r;
      yi = f.q;
    }
    out.resize(o0 + ny);
    float *d = out.data() + o0;
    float lr = _prev.real(), li = _prev.imag();
    if (ny > 0) {
      d[0] = atan2_bf(yi[0] * lr - yr[0] * li, yr[0] * lr + yi[0] * li) * k;
      for (int j = 1; j < ny; j++) {
        const float re = yr[j] * yr[j - 1] + yi[j] * yi[j - 1];
        const float im = yi[j] * yr[j - 1] - yr[j] * yi[j - 1];
        d[j] = atan2_bf(im, re) * k;
      }
      _prev = cf(yr[ny - 1], yi[ny - 1]);
      if (centre != 0)
        for (int j = 0; j < ny; j++) d[j] += centre;
    }
    if (rep) _clicks->envelope(yr, yi, ny);
    xs += m;
    n -= m;
  }
  if (rep) _clicks->mark(out, bad);
}

}  // namespace kp
