// fpvDROID decoder: the luma and colour paths. See picture_filters.h.
#include "picture_filters.h"

#include <algorithm>
#include <cmath>
#include <cstring>

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

// The notch chain: the two sections of luma, U and V, and the notch on luma behind them.
//
// As the luma comb's loop below (which has the reasons, found on a phone): the samples go in runs that lie in one
// stretch of the ring; the first sections of luma, U and V go in one vector with the notch in its fourth lane
// (NotchState::a), their second sections in another (b), one step of each per sample; and the stages are a step
// apart, each taking what the step before left, so that nothing in a step waits for anything else in it: in one
// step the first sections are at sample t, the second at t - 1 and the notch at t - 2. Each lane's arithmetic is
// Biquad::run's, on the same numbers in the same order as when the sections ran one after the other. The loop is
// compiled with colour and without, and takes several steps to a pass.
namespace {

struct NotchState {
  Biquad4 a;  // lanes 0-2: the first sections of luma, U, V; lane 3: the notch
  Biquad4 b;  // lanes 0-2: their second sections
  std::complex<float> osc;
};

struct NotchIn {  // a run of samples
  const float *x;
  float *lr;                // where they go: brightness ...
  std::complex<float> *cr;  // ... and colour
  size_t n;
  size_t renorm;            // the sample at which the oscillator is brought back to length 1 (none: >= n)
  std::complex<float> step;
};

// Lanes [from, to) of a section's state as they were in `was`.
inline void put_back(Biquad4 &q, const Biquad4 &was, int from, int to) {
  for (int i = from; i < to; i++) {
    q.z1[i] = was.z1[i];
    q.z2[i] = was.z2[i];
  }
}

template <bool kCol> void notch_run(NotchState &st, const NotchIn &r) {
  using cf = std::complex<float>;
  Biquad4 a = st.a, b = st.b;
  cf osc = st.osc;
  const cf step = r.step;
  const size_t n = r.n;
  v4f ya_was{}, yb_was{};  // the two vectors' outputs of the step before
  // One step, t = 0 .. n + 1. A: the first sections have a sample (t); B: the second have one (t - 1); N: the
  // notch has one (t - 2).
  const auto one = [&](size_t t, bool A, bool B, bool N) __attribute__((always_inline)) {
    v4f in{};
    if (A) {
      const float x = r.x[t];
      cf c{};
      if (kCol) {
        c = x * std::conj(osc);
        osc *= step;
        if (t == r.renorm) osc /= std::abs(osc);
      }
      in = v4f{x, c.real(), c.imag(), 0.f};
    }
    if (N) in[3] = yb_was[0];
    const Biquad4 a_was = a;
    const v4f ya = a.run(in);
    if (!A) put_back(a, a_was, 0, 3);
    if (!N) put_back(a, a_was, 3, 4);
    if (N) r.lr[t - 2] = ya[3];
    if (B) {
      const v4f yb = b.run(ya_was);  // (its fourth lane, with no section in it, stays nothing)
      if (kCol) r.cr[t - 1] = cf(yb[1], yb[2]);
      yb_was = yb;
    }
    ya_was = ya;
  };
  // (both in line: left as a function of its own, this one would have the loop's filters kept in memory for it)
  const auto edge = [&](size_t t) __attribute__((always_inline)) { one(t, t < n, t >= 1 && t <= n, t >= 2); };
  size_t t = 0;
  for (; t < 2 && t < n + 2; t++) edge(t);
  // (several steps to a pass: what a step hands on then stays where it is from one to the next)
  for (; t + 4 <= n; t += 4) {
    one(t, true, true, true);
    one(t + 1, true, true, true);
    one(t + 2, true, true, true);
    one(t + 3, true, true, true);
  }
  for (; t < n; t++) one(t, true, true, true);
  for (; t < n + 2; t++) edge(t);
  st.a = a;
  st.b = b;
  st.osc = osc;
}

}  // namespace

void PictureFilters::filter_notch(SampleRings &R, const float *d, const uint8_t *bd, size_t cnt) {
  NotchState st{{}, {}, _lo};
  st.a.load(0, _lp_luma);
  st.a.load(1, _cl_r1);
  st.a.load(2, _cl_i1);
  st.a.load(3, _luma_notch);
  st.b.load(0, _lp_luma2);
  st.b.load(1, _cl_r2);
  st.b.load(2, _cl_i2);
  const bool col = _colour;
  const size_t mask = R.mask;
  NotchIn in{};
  in.step = _lo_step;
  uint64_t idx = R.n;
  while (cnt > 0) {
    const size_t at = idx & mask;
    const size_t n = std::min(cnt, mask + 1 - at);
    if (bd) std::memcpy(R.bad.data() + at, bd, n);
    else std::memset(R.bad.data() + at, 0, n);
    in.x = d;
    in.lr = R.luma.data() + at;
    in.cr = R.chroma.data() + at;
    in.n = n;
    in.renorm = static_cast<size_t>((4096 - (idx & 4095)) & 4095);
    if (col) notch_run<true>(st, in);
    else notch_run<false>(st, in);
    d += n;
    if (bd) bd += n;
    idx += n;
    cnt -= n;
  }
  st.a.store(0, _lp_luma);
  st.b.store(0, _lp_luma2);
  st.a.store(3, _luma_notch);
  if (col) {  // without colour its filters stay as they were (only ever fed zeros here)
    st.a.store(1, _cl_r1);
    st.a.store(2, _cl_i1);
    st.b.store(1, _cl_r2);
    st.b.store(2, _cl_i2);
    _lo = st.osc;
  }
}

// The luma comb, and the adaptive comb on top of it.
//
// What a sample costs here, on a phone's cores above all, is less the filters' arithmetic than the number of
// instructions around it: a sample's work is a long chain (wide low-pass, the comb's three filters, the fade, the
// colour's two sections), and a core has only so many instructions in hand at once, so it gets little of the next
// sample started before this one is done. So:
//  - The samples go in runs of up to kCombRun that lie in one stretch of the ring. For a run the line before is
//    read first, at each sample's place between two ring samples (line_before: plain loops over arrays, which the
//    compiler does four samples at a time: NEON on a phone, SSE on a PC), and the click marks are copied.
//  - The eight filter sections go in two vectors of four, one step of each per sample: the wide low-pass with
//    the comb's three filters (CombState::a), and U and V's first sections with their second (c).
//  - The stages are staggered, each a sample or two behind the one that feeds it (comb_run has by how much), and
//    each takes what an earlier step left: so nothing in a step waits for anything else in it, and the core runs
//    the steps of several samples at once. (Stepped together on the same sample, or one sample apart, a vector
//    waits for the slowest thing that feeds any of its lanes, and a sample cost as much as before.) Each lane's
//    arithmetic is Biquad::run's, on the same numbers in the same order as when the sections ran one after the
//    other.
//  - At the two ends of a run a stage has no sample: its lanes are stepped on nothing and put back as they were.
//  - The loop (comb_run) is compiled once for every setting of the switches, with nothing left to ask per sample.
namespace {

constexpr size_t kCombRun = 1024;

struct CombState {  // what the loop carries from sample to sample
  Biquad4 a;  // lanes 0-2: the colour band of the line difference, the adaptive comb's low band and notch; 3: wide
  Biquad4 c;  // lanes 0-1: U and V's first sections; 2-3: their second
  std::complex<float> osc;
  float vd;
  int quiet;
};

struct CombIn {  // a run of samples
  const float *x;              // the samples
  const float *pw, *pl, *pd;   // the line before at each of them: wide, low band, raw (line_before)
  CombTap *tap;                // where they go: the comb's ring ...
  float *lr, *fdr;             // ... brightness, the fade ...
  std::complex<float> *cr;     // ... and colour
  size_t n;
  size_t renorm;               // the sample at which the oscillator is brought back to length 1 (none: >= n)
  std::complex<float> step;
  float idepth;
};

// The line before, at n samples: ring samples p[k] and p[k + 1], fr of the way from the one to the other.
void line_before(const CombTap *p, float fr, size_t n, float *pw, float *pl, float *pd) {
  if (pd) {
    for (size_t k = 0; k < n; k++) {
      pw[k] = p[k].wide + (p[k + 1].wide - p[k].wide) * fr;
      pl[k] = p[k].low + (p[k + 1].low - p[k].low) * fr;
      pd[k] = p[k].d + (p[k + 1].d - p[k].d) * fr;
    }
  } else if (pl) {
    for (size_t k = 0; k < n; k++) {
      pw[k] = p[k].wide + (p[k + 1].wide - p[k].wide) * fr;
      pl[k] = p[k].low + (p[k + 1].low - p[k].low) * fr;
    }
  } else {
    for (size_t k = 0; k < n; k++) pw[k] = p[k].wide + (p[k + 1].wide - p[k].wide) * fr;
  }
}

// kPrev: there is a line before (not in the stream's first line). kAdapt: the adaptive comb; kFull: it has had
// its line to fill (and there is a line before); kComb3: the 3-line chroma comb is on; kCol: colour.
template <bool kPrev, bool kAdapt, bool kFull, bool kComb3, bool kCol> void comb_run(CombState &st, const CombIn &r) {
  using cf = std::complex<float>;
  Biquad4 a = st.a, c = st.c;
  cf osc = st.osc;
  float vd = st.vd;
  int quiet = st.quiet;
  const cf step = r.step;
  const float idepth = r.idepth;
  const size_t n = r.n;
  // What one step hands to the next ones: the wide signal at the last three samples, the comb's filters' output
  // of the step before, the colour ahead of its first sections, and out of them.
  float w1 = 0, w2 = 0, w3 = 0;
  v4f y_was{};
  v2f cin_was{}, c1_was{};
  bool settle1 = false, settle2 = false;  // the colour's sections are to be looked at before their next sample
  // One step, t = 0 .. n + 4. In it the wide low-pass is at sample t (A), the comb's filters at t - 2 (B), the
  // fade, the brightness and the colour's mixing at t - 3 (G), the colour's first sections at t - 4 (C1) and its
  // second at t - 5 (C2): each takes what a step before it left, so that nothing in a step waits for anything
  // else in it.
  const auto one = [&](size_t t, bool A, bool B, bool G, bool C1, bool C2) __attribute__((always_inline)) {
    if (kCol && kAdapt && kFull && !kComb3) {
      if (settle2) {
        c.settle(2);
        c.settle(3);
      }
      settle2 = settle1;
      if (settle1) {
        c.settle(0);
        c.settle(1);
      }
      settle1 = false;
    }
    v4f in{};
    if (B) {
      // NTSC colour flips phase from one line to the next. Half the difference with the same point one line
      // earlier (T samples, tracked) is colour plus vertical detail; band-passing it at the subcarrier leaves
      // the colour, and the brightness is the wide signal minus that. No notch, so no ringing.
      in = v4f{w2, w2, w2, w2};
      in[0] = 0.5f * (w2 - (kPrev ? r.pw[t - 2] : w2));
    }
    if (A) in[3] = r.x[t];
    const Biquad4 a_was = a;
    const v4f y = a.run(in);
    if (!A) put_back(a, a_was, 3, 4);
    if (!B) put_back(a, a_was, 0, 3);
    if (A) {
      r.tap[t].wide = y[3];
      r.tap[t].d = r.x[t];
    }
    v2f cin{};
    if (G) {
      const size_t s = t - 3;
      const float x = r.x[s];
      float csrc = x;  // what the colour is demodulated from
      float luma = w3 - y_was[0];
      if (kAdapt) {
        // Adaptive comb: how different the line before is, below the colour band (so colour itself, which
        // flips from line to line, doesn't count), in sync depths; past ~0.25 fall back to the notch, fully by
        // ~0.6.
        const float lf = y_was[1];
        r.tap[s].low = lf;
        const float ln = y_was[2];
        if (kFull) {
          const float dv = std::fabs(lf - r.pl[s]);
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
          if (kComb3) {
            r.fdr[s] = fade;
          } else {
            csrc = 0.5f * (x - r.pd[s]);
            csrc *= 1.f - fade;
            // While the fade is complete (a whole line under a horizontal edge; the first picture line of every
            // field too: redo_without_comb() works that line's colour out again afterwards, with filters of its
            // own, but this one still runs through the line here) the chroma low-pass is fed exact zeros, and
            // within a few hundred samples its state decays into denormal numbers, which x86 computes some
            // hundred times slower: 9 % of the decode time on the bench's test picture. Looked at every 64
            // samples, it never gets that far. (Both sections before this sample: the first a step on, the
            // second two.)
            if (fade < 1.f) quiet = 0;
            else if ((++quiet & 63) == 0) settle1 = true;
          }
        }
      }
      r.lr[s] = luma;
      if (kCol) {
        const cf cc = csrc * std::conj(osc);
        osc *= step;
        if (s == r.renorm) osc /= std::abs(osc);
        cin = v2f{cc.real(), cc.imag()};
      }
    }
    if (kCol && (C1 || C2)) {
      const Biquad4 c_was = c;
      const v4f yc = c.run(__builtin_shufflevector(cin_was, c1_was, 0, 1, 2, 3));
      if (!C1) put_back(c, c_was, 0, 2);
      if (!C2) put_back(c, c_was, 2, 4);
      if (C2) r.cr[t - 5] = cf(yc[2], yc[3]);
      c1_was = v2f{yc[0], yc[1]};
    }
    w3 = w2;
    w2 = w1;
    w1 = y[3];
    y_was = y;
    cin_was = cin;
  };
  // (both in line: left as a function of its own, this one would have the loop's filters kept in memory for it)
  const auto edge = [&](size_t t) __attribute__((always_inline)) {
    one(t, t < n, t >= 2 && t < n + 2, t >= 3 && t < n + 3, t >= 4 && t < n + 4, t >= 5);
  };
  size_t t = 0;
  for (; t < 5 && t < n + 5; t++) edge(t);
  // Several steps to a pass of the loop: what a step hands on to the next (the filters' states, the wide signal
  // of the last steps) then goes from one to the other where it is, and only once a pass is moved back to where
  // the loop began with it. A step to a pass, a phone's middle cores spent a seventh of the loop on such moves.
  for (; t + 6 <= n; t += 6) {
    one(t, true, true, true, true, true);
    one(t + 1, true, true, true, true, true);
    one(t + 2, true, true, true, true, true);
    one(t + 3, true, true, true, true, true);
    one(t + 4, true, true, true, true, true);
    one(t + 5, true, true, true, true, true);
  }
  for (; t < n; t++) one(t, true, true, true, true, true);
  for (; t < n + 5; t++) edge(t);
  st.a = a;
  st.c = c;
  st.osc = osc;
  st.vd = vd;
  st.quiet = quiet;
}

using CombRunFn = void (*)(CombState &, const CombIn &);
#define KP_COMB_RUNS(P, A, F) comb_run<P, A, F, false, false>, comb_run<P, A, F, false, true>, comb_run<P, A, F, true, false>, comb_run<P, A, F, true, true>
// by kPrev, kAdapt, kFull, kComb3, kCol: 16, 8, 4, 2, 1
const CombRunFn kCombRuns[32] = {KP_COMB_RUNS(false, false, false), KP_COMB_RUNS(false, false, true),
                                 KP_COMB_RUNS(false, true, false),  KP_COMB_RUNS(false, true, true),
                                 KP_COMB_RUNS(true, false, false),  KP_COMB_RUNS(true, false, true),
                                 KP_COMB_RUNS(true, true, false),   KP_COMB_RUNS(true, true, true)};
#undef KP_COMB_RUNS

}  // namespace

void PictureFilters::filter_comb(SampleRings &R, const float *d, const uint8_t *bd, size_t cnt) {
  CombState st{{}, {}, _lo, _vdiff, _chroma_quiet};
  st.a.load(0, _bp_chroma);
  st.a.load(1, _lp_vert);
  st.a.load(2, _luma_notch_w);
  st.a.load(3, _lp_wide);
  st.c.load(0, _cl_r1);
  st.c.load(1, _cl_i1);
  st.c.load(2, _cl_r2);
  st.c.load(3, _cl_i2);
  const bool col = _colour, adapt = _adaptive, adapt_full = _adaptive_lines > 1, comb3 = _comb3;
  const uint64_t back = _prev_back;
  const float fr = _prev_fr;
  const size_t mask = R.mask;
  CombTap *ring = R.comb.data();
  float pw[kCombRun], pl[kCombRun], pd[kCombRun];
  CombIn in{};
  in.pw = pw;
  in.pl = pl;
  in.pd = pd;
  in.step = _lo_step;
  in.idepth = _inv_depth;
  uint64_t idx = R.n;
  while (cnt > 0) {
    const size_t at = idx & mask;
    size_t n = std::min({cnt, kCombRun, mask + 1 - at});
    // the same point on the line before: T samples back, between two ring samples (_prev_back and _prev_fr,
    // worked out once per line: set_line). The stream's first line has none.
    const bool have_prev = idx > back;
    if (!have_prev) {
      n = std::min<size_t>(n, back + 1 - idx);
    } else {
      const bool full = adapt && adapt_full;
      const size_t from = (idx - back) & mask;
      if (from == mask) {  // (the two ring samples are the ring's last and its first)
        n = 1;
        const CombTap two[2] = {ring[mask], ring[0]};
        line_before(two, fr, 1, pw, full ? pl : nullptr, full && !comb3 ? pd : nullptr);
      } else {
        n = std::min(n, mask - from);
        line_before(ring + from, fr, n, pw, full ? pl : nullptr, full && !comb3 ? pd : nullptr);
      }
    }
    if (bd) std::memcpy(R.bad.data() + at, bd, n);
    else std::memset(R.bad.data() + at, 0, n);
    in.x = d;
    in.tap = ring + at;
    in.lr = R.luma.data() + at;
    in.fdr = R.fade.data() + at;
    in.cr = R.chroma.data() + at;
    in.n = n;
    in.renorm = static_cast<size_t>((4096 - (idx & 4095)) & 4095);
    kCombRuns[(have_prev ? 16 : 0) | (adapt ? 8 : 0) | (have_prev && adapt && adapt_full ? 4 : 0) | (comb3 ? 2 : 0) |
              (col ? 1 : 0)](st, in);
    d += n;
    if (bd) bd += n;
    idx += n;
    cnt -= n;
  }
  st.a.store(0, _bp_chroma);
  if (adapt) {  // without the adaptive comb its filters stay as they were
    st.a.store(1, _lp_vert);
    st.a.store(2, _luma_notch_w);
  }
  st.a.store(3, _lp_wide);
  if (col) {
    st.c.store(0, _cl_r1);
    st.c.store(1, _cl_i1);
    st.c.store(2, _cl_r2);
    st.c.store(3, _cl_i2);
    _lo = st.osc;
  }
  _vdiff = st.vd;
  _chroma_quiet = st.quiet;
}

void PictureFilters::redo_without_comb(SampleRings &R, double pic0, double aw, double end, bool whole) {
  // The whole line: from the first good point SparkleRepair::patch_line may take (8 samples ahead of the
  // picture) to its last one in luma (20 after it, and short of `end`; those of its chroma patches further on lie
  // in blanking, where there is no colour either way). The left half: to the middle, the last kBlend samples
  // blended into what the comb made. The filters start kWarm samples ahead, in the sync pulse, and have settled
  // long before the picture.
  constexpr int kWarm = 128, kBlend = 32;
  const int64_t s0 = static_cast<int64_t>(pic0) - 8;
  const int64_t s1 = whole ? std::min(static_cast<int64_t>(pic0 + aw) + 21, static_cast<int64_t>(end))
                           : static_cast<int64_t>(pic0 + 0.5 * aw);
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
