#include "pilots.h"

#include <algorithm>
#include <cmath>

namespace kp {

std::vector<PilotCopy> Pilots::add(const std::vector<Pilot> &found, double keep_mhz) {
  merge(found);
  return drop_copies(keep_mhz);
}

void Pilots::merge(const std::vector<Pilot> &found) {
  for (const auto &f : found) {
    const auto it =
        std::find_if(_list.begin(), _list.end(), [&](const Pilot &p) { return std::fabs(p.mhz - f.mhz) < kSameMhz; });
    if (it == _list.end()) _list.push_back(f);
    else if (f.snr_db >= it->snr_db - kWeakerDb) *it = f;
    else it->seen_ms = f.seen_ms;  // still there, measured better before
  }
  std::stable_sort(_list.begin(), _list.end(), [](const Pilot &a, const Pilot &b) { return a.snr_db > b.snr_db; });
}

std::vector<PilotCopy> Pilots::drop_copies(double keep_mhz) {
  std::vector<Pilot> kept;
  std::vector<PilotCopy> copies;
  for (const auto &p : _list) {
    const auto of = std::find_if(kept.begin(), kept.end(), [&](const Pilot &k) {
      return std::fabs(k.mhz - p.mhz) < kCopyMhz && k.snr_db >= p.snr_db + kCopyDb;
    });
    const bool chosen = keep_mhz > 0 && std::fabs(p.mhz - keep_mhz) < kSameMhz;
    if (of != kept.end() && !chosen) copies.push_back({p.mhz, of->mhz});
    else kept.push_back(p);
  }
  _list.swap(kept);
  return copies;
}

double Pilots::watch_after_scan(double resume_mhz) const {
  double mhz = 0;
  if (resume_mhz > 0)
    for (const auto &p : _list)
      if (std::fabs(p.mhz - resume_mhz) < kSameMhz) mhz = p.mhz;
  if (mhz == 0 && !_list.empty()) mhz = _list.front().mhz;
  if (mhz == 0) mhz = resume_mhz;
  return mhz;
}

}  // namespace kp
