// fpvDROID decoder: sparkle repair. See sparkle.h.
#include "sparkle.h"

#include <algorithm>
#include <cmath>

namespace kp {

SparkleRepair::SparkleRepair(double fs) {
  // How long after a click each filter still carries it: the 1.2 MHz chroma low-pass peaks 0.4 us after it,
  // swings back and has settled by 1.2 us; the adaptive comb's 1 MHz band by 0.6 us; the comb's wide luma within
  // 0.15 us.
  _fwd_c = std::max(1, static_cast<int>(std::lround(1.2e-6 * fs)));
  _fwd_low = std::max(1, static_cast<int>(std::lround(0.6e-6 * fs)));
  _fwd_wide = std::max(1, static_cast<int>(std::lround(0.15e-6 * fs)));
  // Where in that stretch a click is big in chroma: the low-pass's main lobe, 0.3 to 0.75 us after the run's
  // first mark; the swing back after it is small (patch_line: two clicks close together). Not from 0.25 us: a
  // lobe a sample wider at 20 MS/s has more such pairs patched as one, which leaves more pixels wrong.
  _lobe0 = static_cast<int>(std::lround(0.3e-6 * fs));
  _lobe1 = std::min(_fwd_c, static_cast<int>(std::lround(0.75e-6 * fs)));
}

void SparkleRepair::mend_rows(uint8_t *buf, int width, uint64_t field_no, int which, int64_t k, bool hit) {
  if (_mend_field != field_no || _mend_which != which) {
    _mend_field = field_no;
    _mend_which = which;
    _bad_from = -1;
  }
  if (hit) {
    if (_bad_from < 0) _bad_from = k;
    return;
  }
  if (_bad_from >= 1 && k - _bad_from <= 3) {
    const size_t rowb = static_cast<size_t>(width) * 4;
    const uint8_t *top = buf + static_cast<size_t>(_bad_from - 1) * rowb, *bot = buf + static_cast<size_t>(k) * rowb;
    const int span = static_cast<int>(k - (_bad_from - 1));
    for (int64_t r = _bad_from; r < k; r++) {
      const int wb = static_cast<int>(r - (_bad_from - 1)), wt = span - wb;
      uint8_t *dst = buf + static_cast<size_t>(r) * rowb;
      for (size_t x = 0; x < rowb; x++) dst[x] = static_cast<uint8_t>((top[x] * wt + bot[x] * wb + span / 2) / span);
      lines_mended++;
    }
  }
  _bad_from = -1;
}

bool SparkleRepair::patch_line(SampleRings &R, double pic0, double aw, double T, bool luma_comb, bool colour_fades) {
  const int64_t s0 = static_cast<int64_t>(pic0) - 4, s1 = static_cast<int64_t>(pic0 + aw) + 4;
  constexpr int kBack = 3, kFwd = 8, kMaxRun = 64;  // luma: filter spread (samples) either side; longest patch
  // a run may end kFwd past s1, and the furthest patch ends on the sample its own spread after that
  const int reach = kFwd + std::max({kFwd, _fwd_c, _fwd_low, _fwd_wide});
  if (!R.has(s0, s1 + reach, 32)) return false;
  // [a, b) of a ring becomes a straight line between the samples either side of it, a - 1 and b
  const auto ramp = [&](auto &ring, int64_t a, int64_t b) {
    const auto va = ring[(a - 1) & R.mask], vb = ring[b & R.mask];
    const float inv = 1.f / static_cast<float>(b - a + 1);
    for (int64_t q = a; q < b; q++) ring[q & R.mask] = va + (vb - va) * (static_cast<float>(q - a + 1) * inv);
  };
  // [a, b) of one of the comb's taps from the same place two lines earlier (2 T back, between two samples)
  const double up2 = std::ceil(2 * T);
  const int64_t back2 = static_cast<int64_t>(up2);
  const float fr2 = static_cast<float>(up2 - 2 * T);
  const auto two_up = [&](float CombTap::*v, int64_t a, int64_t b) {
    for (int64_t q = a; q < b; q++) {
      const float x0 = R.comb[(q - back2) & R.mask].*v, x1 = R.comb[(q - back2 + 1) & R.mask].*v;
      R.comb[q & R.mask].*v = x0 + (x1 - x0) * fr2;
    }
  };
  const bool taps = luma_comb && R.kept(s0 - kBack - back2);
  // Two clicks close together. A patch ends on a point taken to be good: kFwd samples after its marked run in
  // luma, _fwd_c in chroma. It is not good when another click shows there: the patch would ramp to a wrong
  // value and the next one start from that ramp, a streak as bright as the click it was meant to remove.
  // Such runs are patched as one, from before the first to after the last. A later click shows at the good
  // point
  //  - in luma, when its run begins up to kFwd - rise samples after this one's end: a click shows `rise`
  //    samples after its first mark (which is 2 ahead of it): at once through the comb's wide filter, a
  //    sample later through the notch's low-pass;
  //  - in chroma, when the good point falls in the main lobe of the low-pass's answer to it, _lobe0 to
  //    _lobe1 after its first mark. A click that began earlier than that has swung back by then: little is
  //    left of it, and one longer patch across both does no better. Except when the filter is fading colour
  //    (colour_fades): there a click also fades the colour around it for as long as its low band lasts, and
  //    any click that began in the stretch before the lobe counts as well.
  // chain() gives the end of the last marked run that belongs with the run ending at e: a later run that begins
  // from..upto samples after it, and on from that one's end; e itself for a click on its own. Luma and chroma
  // chain separately, so that no more of either is drawn over than has to be. The comb's taps have no good
  // points to spoil.
  const int rise = luma_comb ? 2 : 3;
  const int c_from = colour_fades ? 0 : _fwd_c - _lobe1, c_upto = _fwd_c - _lobe0;
  const auto chain = [&](int64_t e, int from, int upto) {
    for (;;) {
      const int64_t last = std::min<int64_t>(e + upto, s1 - 1);  // a run begins before s1, as below
      int64_t q = e + from;
      while (q <= last && !(R.bad[q & R.mask] && !R.bad[(q - 1) & R.mask])) q++;
      if (q > last) return e;
      while (q < s1 + kFwd && R.bad[q & R.mask]) q++;
      e = q;
    }
  };
  // ..._end: the end of the chain the last patch covers; ..._solo: the end of a chain too long for one patch
  int64_t lost = 0, luma_end = s0, chroma_end = s0, luma_solo = s0, chroma_solo = s0;
  bool too_long = false, luma_fits = true, chroma_fits = true;
  for (int64_t i = s0; i < s1;) {
    if (!R.bad[i & R.mask]) {
      i++;
      continue;
    }
    int64_t j = i;
    while (j < s1 + kFwd && R.bad[j & R.mask]) j++;
    lost += j - i;
    // The length limit is on the luma patch; chroma's longer spread does not count against it. A chain too
    // long for one patch: in that ring (luma or chroma; the other may still find a chain that fits among the
    // same runs) every run of it is patched on its own (..._solo: up to where). Not left alone: the comb's
    // taps would stay as they are too, and the click would show in the line below. Not a shorter chain from
    // each run in turn: looking for one again from every run is slow on a line full of marks. And the row does
    // not count as badly hit for it, a trade: redrawing the row from its neighbours hides a dense burst (eight
    // clicks 8 samples apart) better than these patches do, but for three or four clicks 20 to 24 apart it
    // costs a row of detail where the short patches do less harm.
    const auto fits = [&](int64_t e) { return e - i + kBack + kFwd <= kMaxRun; };
    if (i >= luma_end) {
      luma_end = j;
      if (i >= luma_solo) {
        const int64_t e = chain(j, 1, kFwd - rise);
        (fits(e) ? luma_end : luma_solo) = e;
      }
      luma_fits = fits(luma_end);
      if (luma_fits) ramp(R.luma, i - kBack, luma_end + kFwd);
    }
    if (i >= chroma_end) {
      chroma_end = j;
      if (i >= chroma_solo) {
        const int64_t e = chain(j, c_from, c_upto);
        (fits(e) ? chroma_end : chroma_solo) = e;
      }
      chroma_fits = fits(chroma_end);
      if (chroma_fits) ramp(R.chroma, i - kBack, chroma_end + _fwd_c);
    }
    if (luma_fits && chroma_fits) {
      if (taps) {
        // The comb's look at the line before: left as it is, the next line would take half the click from
        // here (its luma through the comb, its colour through the adaptive comb's line difference) and
        // show it again one line down, where nothing is marked. That line reads these samples a line
        // period after they came in; this runs well before, as soon as the next line's sync has passed.
        // The stand-in is the line two up, whose colour has the same phase (a straight line would have
        // no colour, and the line below would lose half of its own there); where that line looks
        // different, the adaptive comb sees it in the low band like any other difference.
        two_up(&CombTap::d, i - 1, j + _fwd_wide);
        two_up(&CombTap::wide, i - 1, j + _fwd_wide);
        two_up(&CombTap::low, i - kBack, j + _fwd_low);
      }
      clicks_repaired++;
    } else {
      too_long = true;
    }
    i = j;
  }
  return too_long || lost > 0.08 * aw;
}

void ClickDetector::begin(bool on) {
  if (!on) {
    _env.clear();
    _dtail.clear();
  }
  _hist = _env.size();
}

void ClickDetector::envelope(const float *re, const float *im, int n) {
  const size_t e0 = _env.size();
  _env.resize(e0 + n);
  float *e = _env.data() + e0;
  for (int j = 0; j < n; j++) e[j] = re[j] * re[j] + im[j] * im[j];
}

void ClickDetector::mark(const std::vector<float> &out, std::vector<uint8_t> &bad) {
  if (_env.size() <= _hist) return;
  constexpr float kDip = 0.12f;  // -9 dB
  constexpr size_t kWiden = 2;
  constexpr double kSlip = 0.7, kSlipMax = 1.5;  // turns
  constexpr float kJump = 8.f;  // MHz
  const size_t max_dip = std::max<size_t>(1, static_cast<size_t>(std::lround(0.4e-6 * _fs)));
  const size_t M = _env.size(), N = M - _hist;  // N == out.size()
  double m = 0;
  for (size_t i = _hist; i < M; i++) m += _env[i];
  m /= static_cast<double>(N);
  _pref = _pref <= 0 ? static_cast<float>(m) : _pref + 0.1f * (static_cast<float>(m) - _pref);
  const float thr = kDip * _pref;
  const double turns = 1e6 / _fs;  // MHz x one sample -> turns
  const auto dsc = [&](size_t q) { return q < _hist ? _dtail[q] : out[q - _hist]; };  // alongside _env
  bad.assign(out.size(), 0);
  size_t i = 0;
  while (i < M && _env[i] < thr) i++;  // under way where the kept samples begin: it is longer than a click
  while (i < M) {
    if (_env[i] >= thr) {
      i++;
      continue;
    }
    size_t j = i;
    while (j < M && _env[j] < thr) j++;
    const size_t a = i >= kWiden ? i - kWiden : 0, b = std::min(M, j + kWiden);
    // A dip cut off by the end of this call cannot be told yet and is left alone. The next call sees it whole
    // and marks what it still can: the samples that were this call's have been handed on by then.
    if (j - i <= max_dip && a >= 3 && b + 3 <= M) {
      const float base = (dsc(a - 1) + dsc(a - 2) + dsc(a - 3) + dsc(b) + dsc(b + 1) + dsc(b + 2)) / 6.f;
      double area = 0;
      float lo = 0, hi = 0;
      for (size_t q = a; q < b; q++) {
        const float v = dsc(q) - base;
        area += v;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
      const double slip = std::fabs(area * turns);
      if (slip > kSlip && slip < kSlipMax && (area > 0 ? hi : -lo) > kJump)
        for (size_t q = std::max(a, _hist); q < b; q++) bad[q - _hist] = 1;
    }
    i = j;
  }
  // the end of this call, for the next one: room for a whole dip, its widening and the level either side
  const size_t keep = std::min(M, max_dip + 2 * kWiden + 8);
  _dtail.resize(keep);
  for (size_t q = 0; q < keep; q++) _dtail[q] = dsc(M - keep + q);
  _env.erase(_env.begin(), _env.end() - static_cast<std::ptrdiff_t>(keep));
}

}  // namespace kp
