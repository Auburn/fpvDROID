#include "cut_steps.h"

#include <algorithm>
#include <cmath>

namespace kp {

namespace {
constexpr double kOverlap = 1e6;   // two parts of one signal, seen in two steps, overlap at least this much
constexpr double kSameStep = 5e6;  // an extra step this near one already given is that one (the chip's 5 MHz grid)

double overlap(double a0, double a1, double b0, double b1) { return std::min(a1, b1) - std::max(a0, b0); }
}  // namespace

std::vector<double> scan_plan() {
  constexpr double kStep = 20e6;
  std::vector<double> p;
  for (double c = 5652e6; c <= 5940e6; c += kStep) p.push_back(c);
  for (double c = 5370e6; c <= 5630e6; c += kStep) p.push_back(c);
  return p;
}

void CutSteps::start() {
  _whole.clear();
  _given.clear();
  _pending.clear();
}

// Seen whole in a step of this scan (more than half of the part inside it), or inside a step already given.
bool CutSteps::covered(const Part &p, double edge_hz) const {
  for (const auto &[w0, w1] : _whole)
    if (overlap(p.f0, p.f1, w0, w1) > 0.5 * (p.f1 - p.f0)) return true;
  for (double c : _given)
    if (p.f0 >= c - edge_hz && p.f1 <= c + edge_hz) return true;
  return false;
}

// The step for a signal of which only the end p shows was seen: kReach inside that end.
CutSteps::Extra CutSteps::one_end(const Part &p) {
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

void CutSteps::give(const Extra &e, std::vector<Extra> &out) {
  if (_given.size() >= kMax) return;
  for (double c : _given)
    if (std::fabs(c - e.hz) < kSameStep) return;
  _given.push_back(e.hz);
  out.push_back(e);
}

std::vector<CutSteps::Extra> CutSteps::step(double centre_hz, double edge_hz, const std::vector<Hump> &whole,
                                            const std::vector<CutSignal> &cut, bool extra, bool last) {
  for (const auto &h : whole)
    _whole.emplace_back(centre_hz + h.offset_hz - h.width_hz / 2, centre_hz + h.offset_hz + h.width_hz / 2);
  std::vector<Extra> out;
  if (extra) return out;
  std::vector<Part> now;
  for (const auto &c : cut) now.push_back(Part{centre_hz + c.f0_hz, centre_hz + c.f1_hz, c.cut_hi, c.snr_db});
  std::vector<bool> used(now.size(), false);
  for (const auto &p : _pending) {
    if (covered(p, edge_hz)) continue;  // this step saw it whole after all
    size_t i = 0;
    while (i < now.size() && (used[i] || overlap(p.f0, p.f1, now[i].f0, now[i].f1) < kOverlap)) i++;
    if (i == now.size()) {
      // not seen here: the plan has gone on in another band. (Or it came and went, or it is wider than this step,
      // which then took all of it for the floor: a wasted step, kMax at most, rather than a VTX missed because a
      // crowded step lost it the same way.)
      if (p.snr_db >= kMinSnrDb) give(one_end(p), out);
      continue;
    }
    Part &q = now[i];
    if (q.cut_hi == p.cut_hi) {
      // the same end seen again, cut by the same edge of this step (with the scan's steps 20 MHz apart, only a
      // signal over 23 MHz wide does that): q waits for the next step in p's place
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
    if (e.hi_hz - e.lo_hz <= 2 * kReach && e.snr_db >= kMinSnrDb) give(e, out);
  }
  _pending.clear();
  for (size_t i = 0; i < now.size(); i++) {
    if (used[i] || covered(now[i], edge_hz)) continue;
    if (!last) _pending.push_back(now[i]);
    else if (now[i].snr_db >= kMinSnrDb) give(one_end(now[i]), out);
  }
  return out;
}

}  // namespace kp
