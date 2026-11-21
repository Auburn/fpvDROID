// kepyur decoder: one line of video into a row of pixels. See line_draw.h.
#include "line_draw.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "biquad.h"

namespace kp {

namespace {

// Four pixels as RGBX words (bytes R, G, B, X): draw_line's per-pixel conversion, each of R, G and B (0..1)
// to clamp(z * 255, 0, 255) truncated to a byte.
typedef uint32_t v4u __attribute__((vector_size(16)));
inline v4u byte_of(v4f z) {
  const v4f lo = {0.f, 0.f, 0.f, 0.f}, hi = {255.f, 255.f, 255.f, 255.f};
  z = z * 255.f;
  z = z < lo ? lo : z;
  z = hi < z ? hi : z;
  return __builtin_convertvector(z, v4u);
}
inline v4u rgb_px(v4f r, v4f g, v4f b) { return 0xff000000u | byte_of(b) << 16 | byte_of(g) << 8 | byte_of(r); }
inline v4u grey_px(v4f y) {
  const v4u q = byte_of(y);
  return 0xff000000u | q << 16 | q << 8 | q;
}

// The first cnt of a vector's four lanes to dst.
template <typename V, typename T> inline void store_lanes(T *dst, const V &v, int cnt) {
  if (cnt == 4) std::memcpy(dst, &v, sizeof v);
  else std::memcpy(dst, &v, cnt * sizeof(T));
}

}  // namespace

LineDrawer::LineDrawer(int width, double fs) : _w(width), _fs(fs) {
  // the adaptive comb's fade behind the 3-line comb (comb_weights): the chroma low-pass's main lobe, as three
  // points
  _fade_back = std::max(1, static_cast<int>(std::lround(0.4e-6 * fs)));
  _fade_step = std::max(1, static_cast<int>(std::lround(0.2e-6 * fs)));
}

void LineDrawer::set_std(const VideoStd &s, double comb_w) {
  _a0 = s.a0_us * 1e-6 * _fs;
  _aw = s.aw_us * 1e-6 * _fs;
  _pal = &s == &kPal;
  _comb_w = comb_w;
  // The draw loops take four pixels at a time and may start anywhere (draw), so their last four can reach up
  // to kPad pixels past the row's end: everything they read per pixel has that many more entries, which are
  // worked out like the others and never stored.
  _prev_u.assign(_w + kPad, 0.f);
  _prev_v.assign(_w + kPad, 0.f);
  _px_cm.assign(_w + kPad, 0.f);
  _px_cr.assign(_w + kPad, 0.f);
  _px_ci.assign(_w + kPad, 0.f);
  _px_off.resize(_w + kPad);
  for (int x = 0; x < _w + kPad; x++) _px_off[x] = s.aw_us * 1e-6 * _fs * x / _w;
}

// Without colour: each pixel the luma between the two samples either side, then to bytes. The interpolation and
// the conversion go four pixels at a time as vectors; the samples are fetched one by one (there is no gather).
void LineDrawer::draw_grey(uint32_t *row, const SampleRings &R, double base, float yscale, int x0, int x1,
                           const DrawParams &p) {
  const double *xo = _px_off.data();
  const float *lr = R.luma.data();
  const size_t mask = R.mask;
  const float sg = p.sign, sb = p.sign * p.blank;
  // pixels x .. x + 3; the last four may reach past x1: the first cnt are stored
  for (int x = x0; x < x1; x += 4) {
    const int cnt = std::min(4, x1 - x);
    v4f a, b, f;
    for (int q = 0; q < 4; q++) {
      const double p = base + xo[x + q];
      const int64_t i0 = static_cast<int64_t>(p);
      f[q] = static_cast<float>(p - i0);
      a[q] = lr[i0 & mask];
      b[q] = lr[(i0 + 1) & mask];
    }
    const v4u px = grey_px(sg * (a + (b - a) * f - sb) * yscale);
    store_lanes(row + x, px, cnt);
  }
}

// In colour: as draw_grey. Chroma is interpolated like luma, the comb taken out (comb_on: the comb's terms per
// pixel, worked out for the line first), turned onto the burst's phase and scaled, PAL-D averaged with the line
// before, and converted to RGB.
void LineDrawer::draw_colour(uint32_t *row, const SampleRings &R, double base, float yscale, std::complex<float> rot,
                             float k_uv, bool comb_on, bool pal_v, int x0, int x1, const DrawParams &p) {
  using cf = std::complex<float>;
  const double cdel = p.cdel;
  const bool pal = _pal;
  const double *xo = _px_off.data();
  const float *lr = R.luma.data();
  const cf *chr = R.chroma.data();
  const size_t mask = R.mask;
  if (comb_on) {
    // comb_weights has the comb on every other pixel; in between, the mean of the two either side
    for (int x = x0; x < x1; x++) {
      const int h = x >> 1;
      if (x & 1) {
        _px_cm[x] = 1 - 0.5f * (_comb_m[h] + _comb_m[h + 1]);
        _px_cr[x] = 0.5f * (_comb_o[h].real() + _comb_o[h + 1].real());
        _px_ci[x] = 0.5f * (_comb_o[h].imag() + _comb_o[h + 1].imag());
      } else {
        _px_cm[x] = 1 - _comb_m[h];
        _px_cr[x] = _comb_o[h].real();
        _px_ci[x] = _comb_o[h].imag();
      }
    }
  }
  float *pu = _prev_u.data(), *pv = _prev_v.data();
  const float sg = p.sign, sb = p.sign * p.blank, rr = rot.real(), ri = rot.imag();
  // PAL-D keeps each pixel's U/V for the next line, all four of a group at a time: what the last group puts
  // past x1 is not this line's, and the entries there are put back afterwards.
  float after_u[kPad], after_v[kPad];
  if (pal) {
    std::memcpy(after_u, pu + x1, sizeof after_u);
    std::memcpy(after_v, pv + x1, sizeof after_v);
  }
  // pixels x .. x + 3; the last four may reach past x1: the first cnt are stored
  for (int x = x0; x < x1; x += 4) {
    const int cnt = std::min(4, x1 - x);
    v4f a, b, f, c0r, c0i, c1r, c1i, fc;
    for (int q = 0; q < 4; q++) {
      const double p = base + xo[x + q];
      const int64_t i0 = static_cast<int64_t>(p);
      f[q] = static_cast<float>(p - i0);
      a[q] = lr[i0 & mask];
      b[q] = lr[(i0 + 1) & mask];
      const double pc = p + cdel;
      const int64_t j0 = static_cast<int64_t>(pc);
      fc[q] = static_cast<float>(pc - j0);
      const cf c0 = chr[j0 & mask], c1 = chr[(j0 + 1) & mask];
      c0r[q] = c0.real();
      c0i[q] = c0.imag();
      c1r[q] = c1.real();
      c1i[q] = c1.imag();
    }
    const v4f y = sg * (a + (b - a) * f - sb) * yscale;
    v4f cre = c0r + (c1r - c0r) * fc, cim = c0i + (c1i - c0i) * fc;
    if (comb_on) {
      v4f cm, cr, ci;
      std::memcpy(&cm, &_px_cm[x], sizeof cm);
      std::memcpy(&cr, &_px_cr[x], sizeof cr);
      std::memcpy(&ci, &_px_ci[x], sizeof ci);
      cre = cre * cm - cr;
      cim = cim * cm - ci;
    }
    v4f u = -((cre * rr - cim * ri) * k_uv), v = -((cre * ri + cim * rr) * k_uv);
    if (pal) {
      if (pal_v) v = -v;
      v4f lu, lv;  // the line before
      std::memcpy(&lu, pu + x, sizeof lu);
      std::memcpy(&lv, pv + x, sizeof lv);
      std::memcpy(pu + x, &u, sizeof u);
      std::memcpy(pv + x, &v, sizeof v);
      u = 0.5f * (u + lu);
      v = 0.5f * (v + lv);
    }
    const v4u px = rgb_px(y + 1.140f * v, y - 0.395f * u - 0.581f * v, y + 2.032f * u);
    store_lanes(row + x, px, cnt);
  }
  if (pal) {
    std::memcpy(pu + x1, after_u, sizeof after_u);
    std::memcpy(pv + x1, after_v, sizeof after_v);
  }
}

// The comb for a line, on every other pixel (chroma is 1.2 MHz wide; per pixel costs twice as much for nothing
// visible; draw_colour interpolates in between): chroma becomes c * (1 - _comb_m) - _comb_o. The neighbours sit
// one line period T (Ti whole samples and Tf more) away, turned onto this line's subcarrier phase: the
// oscillator runs on the absolute sample index, so what is mixed down one line away comes out turned by w*T.
void LineDrawer::comb_weights(const SampleRings &R, double ls, float yscale, bool up, bool dn, bool fade,
                              const DrawParams &p) {
  using cf = std::complex<float>;
  const double a0 = _a0, aw = _aw, T = p.T, cdel = p.cdel;
  const int64_t Ti = static_cast<int64_t>(T);
  const float Tf = static_cast<float>(T - Ti);
  const cf turn = std::polar(1.f, static_cast<float>(-_comb_w * T));  // the line above; below: conj
  const float wk = std::fabs(yscale) / (kCombNone - kCombFull), w1 = kCombNone / (kCombNone - kCombFull);
  const int nc = (_w + 1) / 2;
  _comb_m.resize(nc + 1);
  _comb_o.resize(nc + 1);
  const double step = 2 * aw / _w;
  for (int h = 0; h < nc; h++) {
    float m = 0, orr = 0, oi = 0;
    const double p = ls + a0 + step * h, pc = p + cdel;
    const int64_t i0 = static_cast<int64_t>(p + 0.5), j0 = static_cast<int64_t>(pc);
    if (R.has(i0 - Ti, j0 + Ti + 3, 8)) {
      // a neighbour's weight: 1 while its luma here is within kCombFull of this line's, 0 past kCombNone
      const float y0 = R.luma[i0 & R.mask];
      const float wu = up ? std::clamp(w1 - std::fabs(R.luma[(i0 - Ti) & R.mask] - y0) * wk, 0.f, 1.f) : 0.f;
      const float wd = dn ? std::clamp(w1 - std::fabs(R.luma[(i0 + Ti) & R.mask] - y0) * wk, 0.f, 1.f) : 0.f;
      if (wu + wd > 0) {
        // A neighbour turned onto this line has chroma -c and the same luma detail, so c + the neighbours'
        // mean is twice the detail: take out half of it per unit of the best weight (all of it at 1).
        m = 0.5f * std::max(wu, wd);
        const float sc = m / (wu + wd);
        const float fc = static_cast<float>(pc - j0);
        // R.chroma at j + t (0 <= t < 2), in plain floats: branch-free, and std::complex cost more here
        const auto take = [&](int64_t j, float t, float rr, float ri) {
          const int iw = static_cast<int>(t);
          j += iw;
          t -= static_cast<float>(iw);
          const cf a = R.chroma[j & R.mask], b = R.chroma[(j + 1) & R.mask];
          const float xr = a.real() + (b.real() - a.real()) * t, xi = a.imag() + (b.imag() - a.imag()) * t;
          orr += xr * rr - xi * ri;
          oi += xr * ri + xi * rr;
        };
        if (wu > 0) take(j0 - Ti - 1, fc + 1 - Tf, turn.real() * sc * wu, turn.imag() * sc * wu);
        if (wd > 0) take(j0 + Ti, fc + Tf, turn.real() * sc * wd, -turn.imag() * sc * wd);
      }
    }
    if (fade) {
      // Adaptive comb: where this line differs from the one before, colour is faded out, because there
      // the detail that passes for colour cannot be cancelled against that line. The comb above can use
      // the line below as well, so only what it left in (all of it at m = 0: no line matches, a stroke one
      // line tall; none at m = 0.5) is faded. The fade was worked out ahead of the chroma low-pass: seen
      // through that filter (its main lobe, as three points) it is what fading ahead of it would have done.
      const int64_t jf = j0 - _fade_back;
      const float f = 0.25f * (R.fade[(jf - _fade_step) & R.mask] + R.fade[(jf + _fade_step) & R.mask]) +
                      0.5f * R.fade[jf & R.mask];
      const float keep = 1.f - f * (1.f - 2.f * m);
      m = 1.f - (1.f - m) * keep;
      orr *= keep;
      oi *= keep;
    }
    _comb_m[h] = m;
    _comb_o[h] = cf(orr, oi);
  }
  _comb_m[nc] = _comb_m[nc - 1];
  _comb_o[nc] = _comb_o[nc - 1];
}

void LineDrawer::draw(uint32_t *row, const SampleRings &R, const LineInfo &l, bool up, bool dn, const DrawParams &p) {
  const double ls = l.ls, a0 = _a0, aw = _aw, T = p.T, cdel = p.cdel;
  const float white = p.blank + 2.5f * (p.blank - p.tip);
  const float yscale = 1.f / (white - p.blank);
  // chroma -> U/V in luma units: the burst is 20/92.5 (NTSC) or 0.15/0.7 (PAL) of black-to-white, and
  // demodulated chroma comes out as -(U + jV)/2 against a burst of +1/2 (see fpv_sim.py's composite())
  const float k_uv = 0.215f / std::max(l.mag, 1e-6f);
  const std::complex<float> rot = std::polar(1.f, static_cast<float>(-l.phase));
  // a neighbour that is not (or not all) in the rings does not count
  if (dn && !R.written(static_cast<int64_t>(ls + a0 + aw + cdel + T + 2))) dn = false;
  if (up && !R.kept(static_cast<int64_t>(ls + a0 - T), 8)) up = false;
  const bool comb_on = l.col && (up || dn || l.fade);
  if (comb_on) comb_weights(R, ls, yscale, up, dn, l.fade, p);
  // [x0, x1): the pixels whose samples are in the rings (a pixel's first sample only grows with x). The others,
  // a run at either end of the line, are black.
  const double base = ls + a0;
  const auto first = [&](int x) { return static_cast<int64_t>(base + _px_off[x]); };
  int x0 = 0, x1 = _w;
  while (x0 < _w && !R.kept(first(x0), 8)) row[x0++] = 0xff000000u;
  while (x1 > x0 && !R.written(first(x1 - 1) + 8)) row[--x1] = 0xff000000u;
  if (l.col) draw_colour(row, R, base, yscale, rot, k_uv, comb_on, l.pal_v, x0, x1, p);
  else draw_grey(row, R, base, yscale, x0, x1, p);
}

}  // namespace kp
