// fpvDROID decoder: holding the line grid through a weak signal. See line_hold.h.
#include "line_hold.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "log.h"

namespace kp {

LineHold::LineHold(double fs) : _fs(fs) {
  _before = static_cast<int>(std::lround(kFoldBeforeUs * 1e-6 * fs));
  _ref = kRefUs * 1e-6 * fs;
  _fold.shape(static_cast<size_t>(std::lround(kFoldUs * 1e-6 * fs)), fs, 1, false);
}

void LineHold::set_std(double T) {
  _long_ok = false;
  _wide.shape(static_cast<size_t>(T / kWideStep), _fs, kWideStep, true);  // a line of it
  _wide_n = 0;
}

// ---- the transitions --------------------------------------------------------------------------------------

// The grid is somewhere else now: what the fold holds was folded on the old one, and the places to come back to
// are the old one's.
void LineHold::stale() {
  _fold_n = _fold_due = 0;
  _sure_start = _base_start = -1;
}

// The fold takes over.
void LineHold::enter() {
  _weak = true;
  _holds++;
  _weak_lines = _unseen = _fold_due = 0;
  _carry = 0;
  _wide_n = 0;
  _wide_line = 0;
}

void LineHold::acquired() {
  enter();
  _found = 0;   // (no line has found its hsync yet)
  _fold_n = 0;
}

void LineHold::jumped() {
  stale();
  _weak = false;
  _found = ~0ull;
  _fold_good = false;
  _unseen = 0;  // (only ever read while holding, and set again when that begins)
}

void LineHold::restart() {
  jumped();
  _weak_ago = kNever;
}

// ---- a line ------------------------------------------------------------------------------------------------

LineHold::Verdict LineHold::line(const SyncSamples &s, const Line &l) {
  Verdict v{l.next, l.T, std::nullopt, false};
  // Most lines find their hsync, and this one did: the grid as it is now is one to come back to, and one end of
  // the stretch of lines the line period is measured over.
  if ((_found & 1) && !_weak && found64() >= kSure) sure(l.next, static_cast<uint64_t>(l.idx) + 1);
  if (!l.broad) fold_line(s, l.start, l.sign);
  _weak_ago = _weak ? 0 : std::min(_weak_ago + 1, kNever);
  if (!_weak) {
    if (l.lines >= 64 && found64() < kWeak) {
      go_weak(l, v);
      return v;
    }
    if (++_fold_due >= 16) {
      _fold_due = 0;
      const std::optional<Seen> seen = look();
      const double us = 1e-6 * _fs;
      if (seen && found64() >= kClean && seen->pos > kRefMinUs * us && seen->pos < kRefMaxUs * us)
        _ref += std::max(0.05, 1.0 / ++_ref_n) * (seen->pos - _ref);
      _fold_good = seen && std::fabs(seen->pos - _ref) < 0.5 * us;
    }
    return v;
  }
  // (not on pulses of noise: in a dropout, or with the sync's noise too large for a line to find its own)
  if (found64() >= kStrong && !coasting() && single_lines_can(l)) {
    KP_LOGD("video: the lines find their hsync again, after %d lines held by the fold (tip %.2f blank %.2f, noise %.2f)",
            _weak_lines, l.levels.tip, l.levels.blank, std::sqrt(_fold_var));
    _weak = false;
    _fold_due = 0;
    _fold_good = false;
    return v;
  }
  _weak_lines++;
  _unseen++;
  wide_fold(s, l.start, l.sign);
  if (++_fold_due < 4) return v;
  _fold_due = 0;
  const std::optional<Seen> seen = look();
  // Has the signal moved (a block of samples lost on the way here)? Then the whole line's fold has the hsync
  // away from the grid, at two looks 32 lines apart or more. Looked for when the fold at the grid has shown
  // nothing for 128 lines, and every 64 lines whatever it shows: after such a jump a dark edge in the picture
  // can pass for a pulse there, with levels of its own that the kept ones then follow, and the grid stayed on
  // it for good (the hsync, lower than anything in the picture, is the whole line's best pulse all the same).
  if (seen ? _weak_lines % 64 == 0 : _unseen >= 128 && _unseen % 16 == 0) {
    const std::optional<double> off = wide_look(l.T);
    if (off && std::fabs(*off) >= kLookUs * 1e-6 * _fs) {
      if (_wide_line != 0 && l.idx - _wide_line >= 32 && std::fabs(*off - _wide_at) < 0.5e-6 * _fs) {
        KP_LOGI("video: re-phased the line grid by %+.2f us at line %lld of the field, by the fold of the lines (%s)",
                *off / _fs * 1e6, l.in_field, seen ? "what it had at the grid was no hsync" : "no hsync at the grid");
        stale();
        _wide_n = _unseen = 0;
        _wide_line = 0;
        v.rephased = true;
        v.next = l.next + *off;
        return v;
      }
      if (_wide_line == 0 || std::fabs(*off - _wide_at) >= 0.5e-6 * _fs) _wide_at = *off, _wide_line = l.idx;
    } else if (seen) {
      _wide_line = 0;
    }
  }
  if (!seen) return v;
  _unseen = 0;
  const double err = seen->pos - _ref;
  const double c = kSteer * err;
  if (std::fabs(err) < 0.5e-6 * _fs) sure(l.next + c, static_cast<uint64_t>(l.idx) + 1);
  if (_long_ok) v.T = _T_long;
  else v.T = l.T + err * (1.0 / 8192);
  fold_moved(c);
  // (the levels are measured ones from here on: no more of the histogram's)
  v.levels = LevelPair{l.levels.tip + 0.5f * (seen->tip - l.levels.tip), l.levels.blank + 0.5f * (seen->blank - l.levels.blank)};
  v.next = l.next + c;
  return v;
}

// Fewer than half of the lines find their hsync: the fold steers from here on. The lines of the way down here
// may have been pulled about by pulses that were noise, and the line period with them: both go back to what
// they were when most lines still found theirs, if that is not long ago.
void LineHold::go_weak(const Line &l, Verdict &v) {
  enter();
  if (_long_ok) v.T = _T_long;
  const uint64_t at = static_cast<uint64_t>(l.idx) + 1;  // the line that begins at next
  bool back = _long_ok && _sure_start >= 0 && at - _sure_idx <= 256;
  if (back) {  // (not if the fold shows the pulse where the grid is now)
    const std::optional<Seen> seen = look();
    back = !(seen && std::fabs(seen->pos - _ref) < 3e-6 * _fs);
  }
  if (back) {
    v.next = _sure_start + static_cast<double>(at - _sure_idx) * v.T;
    _fold_n = 0;  // what is in the fold was folded on the grid as it was pulled about
  }
  KP_LOGD("video: fewer than half of the lines find their hsync: held by the fold%s (line %lld, tip %.2f blank %.2f)",
          back ? ", from the grid as it was" : "", static_cast<long long>(l.idx), l.levels.tip, l.levels.blank);
}

// The line idx begins at start, and the grid is right there (most lines find their hsync, or the fold shows it):
// a place to come back to, and one end of a long stretch of lines by which the line period is known far better
// than the lines' own tracking has it: (start - where the stretch began) / lines.
void LineHold::sure(double start, uint64_t idx) {
  _sure_start = start;
  _sure_idx = idx;
  if (_base_start < 0 || idx <= _base_idx) {
    _base_start = start;
    _base_idx = idx;
    _mid_start = -1;
    return;
  }
  const uint64_t span = idx - _base_idx;
  if (span >= kLongMin) {
    _T_long = (start - _base_start) / static_cast<double>(span);
    _long_ok = true;
  }
  if (_mid_start < 0 && span >= kLongSpan / 2) _mid_start = start, _mid_idx = idx;
  if (span >= kLongSpan) {  // the stretch moves on, so that the period follows a crystal that drifts
    _base_start = _mid_start;
    _base_idx = _mid_idx;
    _mid_start = start;
    _mid_idx = idx;
  }
}

// ---- the fold around where each line's hsync is due --------------------------------------------------------

// The line that began at ls into the fold.
void LineHold::fold_line(const SyncSamples &s, double ls, float sign) {
  const int64_t i0 = static_cast<int64_t>(std::llround(ls)) - _before;
  const size_t len = _fold.v.size();
  if (!s.kept(i0)) return;
  const float a = 1.f / static_cast<float>(std::min(_fold_n + 1, kFoldLines));
  float sq = 0;
  // (in one stretch of the ring, or two where it wraps: loops the compiler can vectorise)
  const size_t at = static_cast<size_t>(i0) & SyncRing::kMask, first = std::min(len, SyncRing::kMask + 1 - at);
  const float *src = s.ring.v.data() + at;
  float *f = _fold.v.data();
  for (size_t j = 0; j < first; j++) {
    const float d = sign * src[j] - f[j];
    sq += d * d;
    f[j] += a * d;
  }
  src = s.ring.v.data();
  f += first;
  for (size_t j = 0; j < len - first; j++) {
    const float d = sign * src[j] - f[j];
    sq += d * d;
    f[j] += a * d;
  }
  sq /= static_cast<float>(len);
  _fold_var = _fold_n == 0 ? 0 : _fold_n == 1 ? sq : _fold_var + std::max(a, 1.f / 16) * (sq - _fold_var);
  _fold_n = std::min(_fold_n + 1, kFoldLines);
}

// Is there an hsync in the fold? One by its shape (Fold::pulse), lower than the porches before and behind it by
// more than the fold's noise would make it anywhere.
std::optional<LineHold::Seen> LineHold::look() {
  if (_fold_n < kFoldMin) return std::nullopt;
  const double us = 1e-6 * _fs;
  const int w = _fold.bins(kLookUs);
  const std::optional<FoldPulse> p = _fold.pulse(_before - w, _before + w);
  const float noise = fold_noise();
  if (!p || p->depth < 2.5f * noise) return std::nullopt;
  const double pos = p->edge - _before;
  // (a pulse that is not where the grid has the hsync has to stand out further: in noise alone the fold shows
  // one somewhere in its 9 us every second or two, by chance)
  if (std::fabs(pos - _ref) > kTightUs * us && p->depth < 4 * noise) return std::nullopt;
  return Seen{pos, p->tip, p->blank};
}

// The noise in one place of a fold of single samples over kFoldLines lines
float LineHold::fold_noise() const {
  return std::sqrt(_fold_var / static_cast<float>(_fold_n < kFoldLines ? std::max(_fold_n, 1) : 2 * kFoldLines));
}

// The grid has been moved by c samples: so has where the fold has everything, the other way.
void LineHold::fold_moved(double c) {
  _carry += c;
  const int k = static_cast<int>(_carry);
  if (k == 0) return;
  _carry -= k;
  std::vector<float> &fold = _fold.v;
  const int len = static_cast<int>(fold.size());
  if (std::abs(k) >= len) {
    _fold_n = 0;
  } else if (k > 0) {
    std::memmove(fold.data(), fold.data() + k, static_cast<size_t>(len - k) * sizeof(float));
    std::fill(fold.end() - k, fold.end(), fold[static_cast<size_t>(len - k - 1)]);
  } else {
    std::memmove(fold.data() - k, fold.data(), static_cast<size_t>(len + k) * sizeof(float));
    std::fill(fold.begin(), fold.begin() - k, fold[static_cast<size_t>(-k)]);
  }
}

// ---- the fold of the whole line ----------------------------------------------------------------------------

void LineHold::wide_fold(const SyncSamples &s, double ls, float sign) {
  std::vector<float> &wide = _wide.v;
  const size_t K = wide.size();
  const int64_t i0 = static_cast<int64_t>(std::llround(ls));
  if (!s.has(i0, i0 + static_cast<int64_t>(K) * kWideStep)) return;
  const float a = 1.f / static_cast<float>(std::min(_wide_n + 1, kFoldLines));
  for (size_t k = 0; k < K; k++)
    wide[k] += a * (sign * s.ring.v[static_cast<size_t>(i0 + static_cast<int64_t>(k) * kWideStep) & SyncRing::kMask] - wide[k]);
  _wide_n = std::min(_wide_n + 1, kFoldLines);
}

// How far from the grid the whole line's fold has an hsync, samples (the nearer way round); nothing: none.
std::optional<double> LineHold::wide_look(double T) {
  if (_wide_n < kFoldLines / 2) return std::nullopt;
  const std::optional<FoldPulse> p = _wide.pulse(0, static_cast<int>(_wide.v.size()) - 1);
  const float noise = std::sqrt(_fold_var / static_cast<float>(2 * kFoldLines));
  if (!p || p->depth < 4 * noise || !p->lowest_there()) return std::nullopt;
  return std::remainder(p->edge * kWideStep - _ref, T);
}

}  // namespace kp
