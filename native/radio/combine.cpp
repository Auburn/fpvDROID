// Two-antenna video (maximal-ratio combining of path A and path B): see combine.h. From Play RX
// (github.com/scerstt/PlayRX, GPL-2.0).
#include "combine.h"

#include <algorithm>
#include <cmath>

namespace kp {

void Combiner::reset() {
  *this = Combiner{};
}

void Combiner::process(cf *a, const cf *b, size_t n) {
  while (n > 0) {
    const size_t m = std::min(n, kBlock);
    block(a, b, m);
    a += m;
    b += m;
    n -= m;
  }
}

void Combiner::block(cf *a, const cf *b, size_t n) {
  if (n == 0) return;
  // DC (the adapter's DC line), smoothed across blocks like the demodulator does
  double ar = 0, ai = 0, br = 0, bi = 0;
  for (size_t i = 0; i < n; i++) {
    ar += a[i].real();
    ai += a[i].imag();
    br += b[i].real();
    bi += b[i].imag();
  }
  const cf ma(static_cast<float>(ar / n), static_cast<float>(ai / n)), mb(static_cast<float>(br / n), static_cast<float>(bi / n));
  if (!_init) _dca = ma, _dcb = mb;
  _dca += (ma - _dca) * 0.3f;
  _dcb += (mb - _dcb) * 0.3f;
  // moments and the cross-correlation
  double m2a = 0, m4a = 0, m2b = 0, m4b = 0, cr = 0, ci = 0;
  for (size_t i = 0; i < n; i++) {
    const float xr = a[i].real() - _dca.real(), xi = a[i].imag() - _dca.imag();
    const float yr = b[i].real() - _dcb.real(), yi = b[i].imag() - _dcb.imag();
    const float pa = xr * xr + xi * xi, pb = yr * yr + yi * yi;
    m2a += pa;
    m4a += static_cast<double>(pa) * pa;
    m2b += pb;
    m4b += static_cast<double>(pb) * pb;
    cr += xr * yr + xi * yi;  // a * conj(b)
    ci += xi * yr - xr * yi;
  }
  m2a /= n, m4a /= n, m2b /= n, m4b /= n, cr /= n, ci /= n;
  const double k = _init ? 0.3 : 1.0;  // ~3 blocks (~150 us at 20 MS/s): far quicker than a fade
  _m2a += (m2a - _m2a) * k;
  _m4a += (m4a - _m4a) * k;
  _m2b += (m2b - _m2b) * k;
  _m4b += (m4b - _m4b) * k;
  _c += (std::complex<double>(cr, ci) - _c) * k;
  // signal and noise of each path (M2M4); noise floored at the 12-bit quantisation and 40 dB SNR
  auto split = [](double m2, double m4, double &s, double &nz) {
    s = std::sqrt(std::max(0.0, 2 * m2 * m2 - m4));
    s = std::min(s, m2);
    nz = std::max({m2 - s, 1e-4 * m2, 0.5});
  };
  double sa, na, sb, nb;
  split(_m2a, _m4a, sa, na);
  split(_m2b, _m4b, sb, nb);
  // the receiver noise hardly changes at a fixed gain, while each block's estimate of it is jumpy when the signal is
  // strong (a small difference of two big numbers): average it over ~5 ms; the signal itself is tracked fast
  const double kn = _init ? 0.01 : 1.0;
  _noise_a += (na - _noise_a) * kn;
  _noise_b += (nb - _noise_b) * kn;
  na = _noise_a;
  nb = _noise_b;
  _init = true;
  const double ca = std::abs(_c);
  const std::complex<double> rot = ca > 0 ? _c / ca : std::complex<double>(1, 0);  // turns B into step with A
  double wa = std::sqrt(sa) / na, wb = std::sqrt(sb) / nb;
  if (wa + wb <= 0) wa = 1, wb = 0;  // nothing on either: A as it is
  const double tot = wa + wb;
  const cf wa_new(static_cast<float>(wa / tot), 0.f);
  const cf wb_new(static_cast<float>(wb / tot * rot.real()), static_cast<float>(wb / tot * rot.imag()));
  // combine, the weights moving linearly from the last block's to this one's
  const cf dwa = (wa_new - _wa) / static_cast<float>(n), dwb = (wb_new - _wb) / static_cast<float>(n);
  cf ua = _wa, ub = _wb;
  for (size_t i = 0; i < n; i++) {
    ua += dwa;
    ub += dwb;
    const cf x = a[i] - _dca, y = b[i] - _dcb;
    a[i] = ua * x + ub * y;
  }
  _wa = wa_new;
  _wb = wb_new;
  // stats
  const double snra = sa / na, snrb = sb / nb, snrc = snra + snrb;
  _sa += snra;
  _sb += snrb;
  _sc += snrc;
  _share += snrc > 0 ? snrb / snrc : 0;
  _coh += std::sqrt(_m2a * _m2b) > 0 ? ca / std::sqrt(_m2a * _m2b) : 0;
  const double thr = std::pow(10.0, kFadeDb / 10);
  if (snra < thr) {
    _fade++;
    if (snrc >= thr) _resc++;
  }
  _nb++;
}

Combiner::Stats Combiner::take_stats() {
  Stats s;
  if (_nb == 0) return s;
  auto db = [](double v) { return static_cast<float>(10 * std::log10(std::max(v, 1e-10))); };
  s.snr_a_db = db(_sa / _nb);
  s.snr_b_db = db(_sb / _nb);
  s.snr_c_db = db(_sc / _nb);
  s.b_share = static_cast<float>(_share / _nb);
  s.coherence = static_cast<float>(_coh / _nb);
  s.a_fade = static_cast<float>(static_cast<double>(_fade) / _nb);
  s.rescued = _fade ? static_cast<float>(static_cast<double>(_resc) / _fade) : 0.f;
  _sa = _sb = _sc = _share = _coh = 0;
  _nb = _fade = _resc = 0;
  return s;
}

}  // namespace kp
