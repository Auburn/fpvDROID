#include "scan.h"

#include <algorithm>
#include <cmath>

namespace kp {

namespace {
constexpr double kOverlap = 1e6;  // two parts of one signal, seen in two steps, overlap at least this much

double overlap(double a0, double a1, double b0, double b1) { return std::min(a1, b1) - std::max(a0, b0); }
}  // namespace

std::vector<double> ScanPlan::plan() {
  std::vector<double> p;
  for (double c = 5652e6; c <= 5940e6; c += kStep) p.push_back(c);
  for (double c = 5370e6; c <= 5630e6; c += kStep) p.push_back(c);
  return p;
}

std::vector<double> ScanPlan::hz_of(const std::vector<Extra> &extra) {
  std::vector<double> hz;
  for (const auto &e : extra) hz.push_back(e.hz);
  return hz;
}

std::vector<double> ScanPlan::start() {
  _plan = plan();
  _at = 0;
  _owed = 0;
  _whole.clear();
  _given.clear();
  _pending.clear();
  return _plan;
}

// Seen whole in a step of this scan (more than half of the part inside it), or inside a step already given.
bool ScanPlan::covered(const Part &p) const {
  for (const auto &[w0, w1] : _whole)
    if (overlap(p.f0, p.f1, w0, w1) > 0.5 * (p.f1 - p.f0)) return true;
  for (double c : _given)
    if (p.f0 >= c - kEdge && p.f1 <= c + kEdge) return true;
  return false;
}

// The step for a signal of which only the end p shows was seen: kReach inside that end.
ScanPlan::Extra ScanPlan::one_end(const Part &p) {
  Extra e;
  e.snr_db = p.snr_db;
  if (p.cut_hi) {
    e.lo_hz = p.f0;
    e.hz = p.f0 + kReach;
  } else {
    e.hi_hz = p.f1;
    e.hz = p.f1 - kReach;
  }
  return e;
}

// At most kMax, and none within a kGrid of one already given (the chip would take the same hardware centre).
void ScanPlan::give(const Extra &e, std::vector<Extra> &out) {
  if (_given.size() >= kMax) return;
  for (double c : _given)
    if (std::fabs(c - e.hz) < kGrid) return;
  _given.push_back(e.hz);
  out.push_back(e);
}

std::vector<ScanPlan::Extra> ScanPlan::step(double centre_hz, const std::vector<Hump> &whole,
                                            const std::vector<CutSignal> &cut) {
  for (const auto &h : whole)
    _whole.emplace_back(centre_hz + h.offset_hz - h.width_hz / 2, centre_hz + h.offset_hz + h.width_hz / 2);
  std::vector<Extra> out;
  if (_owed > 0) {  // an extra step: what it sees whole counts, what it sees cut asks for nothing
    _owed--;
    return out;
  }
  // only the plan's next step kStep up can see more of what this one cuts: past the last step of a band, none does
  const bool next = _at + 1 < _plan.size() && std::fabs(_plan[_at + 1] - _plan[_at] - kStep) < 1;
  _at++;
  std::vector<Part> now;
  for (const auto &c : cut) now.push_back(Part{centre_hz + c.f0_hz, centre_hz + c.f1_hz, c.cut_hi, c.snr_db});
  std::vector<bool> used(now.size(), false);
  for (const auto &p : _pending) {
    if (covered(p)) continue;  // this step saw it whole after all
    size_t i = 0;
    while (i < now.size() && (used[i] || overlap(p.f0, p.f1, now[i].f0, now[i].f1) < kOverlap)) i++;
    if (i == now.size()) {
      // not seen here: cut by the plan's step before at its bottom edge, it ends below this one (or it came and
      // went, or it is wider than this step, which then took all of it for the floor: a wasted step, kMax at most,
      // rather than a VTX missed because a crowded step lost it the same way)
      if (p.snr_db >= kMinSnrDb) give(one_end(p), out);
      continue;
    }
    Part &q = now[i];
    if (q.cut_hi == p.cut_hi) {
      // the same end seen again, cut by the same edge of this step (with steps kStep apart, only a signal over 23 MHz
      // wide does that): q waits for the next step in p's place
      q.snr_db = std::max(q.snr_db, p.snr_db);
      continue;
    }
    // its other end, cut by this step's other edge: the whole signal
    used[i] = true;
    const Part &bottom = p.cut_hi ? p : q, &top = p.cut_hi ? q : p;
    Extra e;
    e.lo_hz = bottom.f0;
    e.hi_hz = top.f1;
    e.hz = (e.lo_hz + e.hi_hz) / 2;
    e.snr_db = std::max(p.snr_db, q.snr_db);
    if (e.hi_hz - e.lo_hz <= kHumpMaxHz && e.snr_db >= kMinSnrDb) give(e, out);
  }
  _pending.clear();
  for (size_t i = 0; i < now.size(); i++) {
    if (used[i] || covered(now[i])) continue;
    if (next) _pending.push_back(now[i]);
    else if (now[i].snr_db >= kMinSnrDb) give(one_end(now[i]), out);
  }
  _owed = out.size();
  return out;
}

}  // namespace kp
