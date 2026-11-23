// fpvDROID decoder: the luma and colour paths. See picture_filters.h.
#include "picture_filters.h"

#include <algorithm>
#include <cmath>

namespace kp {

PictureFilters::PictureFilters(double fs, bool colour) : _fs(fs), _colour(colour) {
  for (Biquad *b : {&_cl_r1, &_cl_i1}) b->design(1.2e6, fs, 0.54);
  for (Biquad *b : {&_cl_r2, &_cl_i2}) b->design(1.2e6, fs, 1.31);
  _lp_luma.design(3.0e6, fs, 0.54);
  _lp_luma2.design(3.0e6, fs, 1.31);
  // luma comb: one gentle section (Q 0.6: no peaking, ~1 % overshoot), -3 dB near 4 MHz
  _lp_wide.design(4.8e6, fs, 0.6);
  _lp_vert.design(1.0e6, fs);  // adaptive comb: brightness below the colour band, line against line
  // cdel(): how much faster the comb's luma filter is than the notch chain's
  const auto delay = [](Biquad a, Biquad *b) {  // group delay at low frequency: the impulse's centroid
    double s = 0, sw = 0;
    for (int i = 0; i < 2048; i++) {
      float x = i == 0 ? 1.f : 0.f;
      x = a.run(x);
      if (b) x = b->run(x);
      s += x;
      sw += x * static_cast<double>(i);
    }
    return s != 0 ? sw / s : 0;
  };
  Biquad l2 = _lp_luma2;
  _cdel_comb = 0.21e-6 * fs + (delay(_lp_luma, &l2) - delay(_lp_wide, nullptr));
}

void PictureFilters::set_std(const VideoStd &s) {
  const double fsc = &s == &kPal ? 4.43361875e6 : 315e6 / 88;
  const double a = 2 * M_PI * fsc / _fs;
  _lo_step = cf(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
  _luma_notch.notch(fsc, _fs, 1.5);  // the subcarrier out of the picture (also the B/W herringbone)
  _luma_notch_w.notch(fsc, _fs, 1.5);  // adaptive comb: the same, on the comb's wide signal
  _bp_chroma.bandpass(fsc, _fs, 1.0);  // luma comb: the colour band of the line difference
  _comb_w = a;
}

void PictureFilters::set_line(bool luma_comb, bool adaptive, bool comb3, double T, float inv_depth) {
  _luma_comb = luma_comb;
  _adaptive = adaptive;
  _comb3 = comb3;
  _adaptive_lines = adaptive ? std::min(_adaptive_lines + 1, 2) : 0;
  // the same point on the line before: n - T = (n - ceil(T)) + (ceil(T) - T)
  const double up = std::ceil(T);
  _prev_back = static_cast<uint64_t>(std::max(up, 1.0));
  _prev_fr = static_cast<float>(up - T);
  _inv_depth = inv_depth;
}

void PictureFilters::run(SampleRings &R, const float *d, const uint8_t *bad, size_t cnt) {
  if (_luma_comb) filter_comb(R, d, bad, cnt);
  else filter_notch(R, d, bad, cnt);
  R.n += cnt;
}

// The notch chain. The two sections of luma, U and V go side by side (Biquad4: lanes 0-2, the first sections in
// s1 and the second in s2, so s1's output is s2's input lane for lane), the notch on its own. All in locals for
// the loop, so that their state stays in registers.
void PictureFilters::filter_notch(SampleRings &R, const float *d, const uint8_t *bd, size_t cnt) {
  Biquad4 s1, s2;
  s1.load(0, _lp_luma);
  s1.load(1, _cl_r1);
  s1.load(2, _cl_i1);
  s2.load(0, _lp_luma2);
  s2.load(1, _cl_r2);
  s2.load(2, _cl_i2);
  Biquad nt = _luma_notch;
  cf osc = _lo;
  const cf step = _lo_step;
  const bool col = _colour;
  const size_t mask = R.mask;
  float *lr = R.luma.data();
  cf *cr = R.chroma.data();
  uint8_t *br = R.bad.data();
  uint64_t idx = R.n;
  for (size_t k = 0; k < cnt; k++, idx++) {
    const size_t at = idx & mask;
    const float x = d[k];
    br[at] = bd ? bd[k] : 0;
    cf c{};
    if (col) {
      c = x * std::conj(osc);
      osc *= step;
      if ((idx & 4095) == 0) osc /= std::abs(osc);
    }
    const v4f y = s2.run(s1.run(v4f{x, c.real(), c.imag(), 0.f}));
    lr[at] = nt.run(y[0]);
    if (col) cr[at] = cf(y[1], y[2]);
  }
  s1.store(0, _lp_luma);
  s2.store(0, _lp_luma2);
  _luma_notch = nt;
  if (col) {  // without colour its filters stay as they were (only ever fed zeros here)
    s1.store(1, _cl_r1);
    s1.store(2, _cl_i1);
    s2.store(1, _cl_r2);
    s2.store(2, _cl_i2);
    _lo = osc;
  }
}

// The luma comb, and the adaptive comb on top of it. Locals as in filter_notch, and side by side the same way:
// after the wide low-pass, its three filters (the colour band of the line difference, and the adaptive comb's
// low band and notch) in s2's lanes 0-2, then U and V's two sections in c1 and c2.
void PictureFilters::filter_comb(SampleRings &R, const float *d, const uint8_t *bd, size_t cnt) {
  Biquad wide = _lp_wide;
  Biquad4 s2;
  s2.load(0, _bp_chroma);
  s2.load(1, _lp_vert);
  s2.load(2, _luma_notch_w);
  Biquad2 c1, c2;
  c1.load(0, _cl_r1);
  c1.load(1, _cl_i1);
  c2.load(0, _cl_r2);
  c2.load(1, _cl_i2);
  cf osc = _lo;
  const cf step = _lo_step;
  const bool col = _colour, adapt = _adaptive, adapt_full = _adaptive_lines > 1, comb3 = _comb3;
  const uint64_t back = _prev_back;
  const float fr = _prev_fr, idepth = _inv_depth;
  float vd = _vdiff;
  int quiet = _chroma_quiet;
  const size_t mask = R.mask;
  float *lr = R.luma.data(), *fdr = R.fade.data();
  cf *cr = R.chroma.data();
  uint8_t *br = R.bad.data();
  CombTap *ring = R.comb.data();
  uint64_t idx = R.n;
  for (size_t k = 0; k < cnt; k++, idx++) {
    const size_t at = idx & mask;
    const float x = d[k];
    br[at] = bd ? bd[k] : 0;
    float csrc = x;  // what the colour is demodulated from
    // NTSC colour flips phase from one line to the next. Half the difference with the same point one line
    // earlier (T samples, tracked) is colour plus vertical detail; band-passing it at the subcarrier leaves
    // the colour, and the brightness is the wide signal minus that. No notch, so no ringing.
    const float w = wide.run(x);
    CombTap &now_tap = ring[at];
    now_tap.wide = w;
    now_tap.d = x;
    // the same point on the line before: T samples back, between two ring samples (_prev_back and _prev_fr,
    // worked out once per line: set_line)
    const bool have_prev = idx > back;
    const CombTap &p0 = ring[(idx - back) & mask], &p1 = ring[(idx - back + 1) & mask];
    const v4f y2 = s2.run(v4f{0.5f * (w - (have_prev ? p0.wide + (p1.wide - p0.wide) * fr : w)), w, w, 0.f});
    float luma = w - y2[0];
    if (adapt) {
      // Adaptive comb: how different the line before is, below the colour band (so colour itself, which
      // flips from line to line, doesn't count), in sync depths; past ~0.25 fall back to the notch, fully by
      // ~0.6.
      const float lf = y2[1];
      now_tap.low = lf;
      const float ln = y2[2];
      if (have_prev && adapt_full) {
        const float dv = std::fabs(lf - (p0.low + (p1.low - p0.low) * fr));
        vd += (dv - vd) * 0.35f;  // ~3 samples: a whole edge, not one noisy sample
        const float fade = std::clamp((vd * idepth - 0.25f) * (1.f / 0.35f), 0.f, 1.f);
        luma += (ln - luma) * fade;
        // And the colour combed too: half the difference with the line before keeps the colour (it flips
        // from line to line) and cancels brightness detail that continues down the picture (the strokes of
        // OSD characters); what is left at a sharp edge is faded out. With the 3-line chroma comb on, that
        // comb does the cancelling (centred on this line), and the fade is only noted here: LineDrawer
        // applies it where the comb finds no line to cancel with. Fading here would take the colour off the
        // line under every horizontal edge, which the comb draws well from the line below, and off part of
        // the next line, which is combed with that one.
        if (comb3) {
          fdr[at] = fade;
        } else {
          csrc = 0.5f * (x - (p0.d + (p1.d - p0.d) * fr));
          csrc *= 1.f - fade;
          // While the fade is complete (a whole line under a horizontal edge; the first picture line of every
          // field too: redo_without_comb() works that line's colour out again afterwards, with filters of its
          // own, but this one still runs through the line here) the chroma low-pass is fed exact zeros, and
          // within a few hundred samples its state decays into denormal numbers, which x86 computes some
          // hundred times slower: 9 % of the decode time on the bench's test picture. Looked at every 64
          // samples, it never gets that far.
          if (fade < 1.f) {
            quiet = 0;
          } else if ((++quiet & 63) == 0) {
            c1.settle(0);
            c1.settle(1);
            c2.settle(0);
            c2.settle(1);
          }
        }
      }
    }
    lr[at] = luma;
    if (col) {
      const cf c = csrc * std::conj(osc);
      osc *= step;
      if ((idx & 4095) == 0) osc /= std::abs(osc);
      const v2f yc = c2.run(c1.run(v2f{c.real(), c.imag()}));
      cr[at] = cf(yc[0], yc[1]);
    }
  }
  _lp_wide = wide;
  s2.store(0, _bp_chroma);
  if (adapt) {  // without the adaptive comb its filters stay as they were
    s2.store(1, _lp_vert);
    s2.store(2, _luma_notch_w);
  }
  if (col) {
    c1.store(0, _cl_r1);
    c1.store(1, _cl_i1);
    c2.store(0, _cl_r2);
    c2.store(1, _cl_i2);
    _lo = osc;
  }
  _vdiff = vd;
  _chroma_quiet = quiet;
}

void PictureFilters::redo_without_comb(SampleRings &R, double pic0, double aw, bool whole) {
  // The whole line: from the first good point SparkleRepair::patch_line may take (8 samples ahead of the
  // picture) to its last one in luma (20 after it; those of its chroma patches further on lie in blanking, where
  // there is no colour either way). The left half: to the middle, the last kBlend samples blended into what the
  // comb made. The filters start kWarm samples ahead, in the sync pulse, and have settled long before the
  // picture.
  constexpr int kWarm = 128, kBlend = 32;
  const int64_t s0 = static_cast<int64_t>(pic0) - 8;
  const int64_t s1 = whole ? static_cast<int64_t>(pic0 + aw) + 21 : static_cast<int64_t>(pic0 + 0.5 * aw);
  const int64_t back = static_cast<int64_t>(R.n) - (s0 - kWarm);
  if (!R.has(s0 - kWarm, s1, 32)) return;
  const auto fresh = [](Biquad b) {
    b.z1 = b.z2 = 0;
    return b;
  };
  Biquad nt = fresh(_luma_notch_w), r1 = fresh(_cl_r1), r2 = fresh(_cl_r2), i1 = fresh(_cl_i1), i2 = fresh(_cl_i2);
  // Colour: only the adaptive comb on its own takes it from the line difference and fades it (filter_comb).
  // Otherwise R.chroma already has it straight from the signal; with the 3-line comb the fade is only noted, and
  // LineDrawer uses it where that comb finds no matching line below either, which is left as it is. The
  // oscillator turns by _comb_w per sample: back to where it was at the first sample.
  const bool chroma = _colour && colour_fades();
  const std::complex<double> was = std::polar(1.0, -_comb_w * static_cast<double>(back));
  cf osc = _lo * cf(static_cast<float>(was.real()), static_cast<float>(was.imag()));
  // filter_comb's line difference reads the line before between two samples, which takes 3 % off the subcarrier
  // and turns it by half a degree. The burst went through it and the picture's colour is measured against
  // the burst, so this line's colour gets the same, or it would come out that much more saturated than the
  // rest: half of (1 - the subcarrier as read _prev_back samples back, _prev_fr of the way to the next sample).
  const std::complex<double> prev = std::polar(1.0, -_comb_w * static_cast<double>(_prev_back)) *
                                    ((1.0 - _prev_fr) + static_cast<double>(_prev_fr) * std::polar(1.0, _comb_w));
  const cf gain(static_cast<float>(0.5 * (1.0 - prev.real())), static_cast<float>(-0.5 * prev.imag()));
  for (int64_t i = s0 - kWarm; i < s1; i++) {
    const CombTap &t = R.comb[i & R.mask];
    const float y = nt.run(t.wide);
    const float mix = whole ? 1.f : std::min(1.f, static_cast<float>(s1 - i) * (1.f / kBlend));
    if (i >= s0) R.luma[i & R.mask] += (y - R.luma[i & R.mask]) * mix;
    if (!chroma) continue;
    const cf c = t.d * std::conj(osc);
    osc *= _lo_step;
    const cf uv(r2.run(r1.run(c.real())), i2.run(i1.run(c.imag())));
    if (i >= s0) R.chroma[i & R.mask] += (uv * gain - R.chroma[i & R.mask]) * mix;
  }
}

}  // namespace kp
