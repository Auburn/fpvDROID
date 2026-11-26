#include "pilots.h"

#include <algorithm>
#include <cmath>

namespace kp {

std::vector<PilotCopy> add_sightings(std::vector<Pilot> &list, const std::vector<Pilot> &found, int64_t scan_ms,
                                     double keep_mhz) {
  for (const auto &f : found) {
    const auto it =
        std::find_if(list.begin(), list.end(), [&](const Pilot &p) { return std::fabs(p.mhz - f.mhz) < kSameMhz; });
    if (it == list.end()) list.push_back(f);
    else if (it->seen_ms < scan_ms || f.snr_db >= it->snr_db - kWeakerDb) *it = f;
    else it->seen_ms = f.seen_ms;
  }
  std::stable_sort(list.begin(), list.end(), [](const Pilot &a, const Pilot &b) { return a.snr_db > b.snr_db; });
  const auto now = [&](const Pilot &p) { return p.seen_ms >= scan_ms; };
  std::vector<Pilot> kept;
  std::vector<PilotCopy> copies;
  for (const auto &p : list) {
    const bool chosen = keep_mhz > 0 && std::fabs(p.mhz - keep_mhz) < kSameMhz;
    const auto of = std::find_if(kept.begin(), kept.end(), [&](const Pilot &k) {
      return now(k) && std::fabs(k.mhz - p.mhz) < kCopyMhz && k.snr_db >= p.snr_db + kCopyDb;
    });
    if (now(p) && !chosen && of != kept.end()) copies.push_back({p.mhz, of->mhz});
    else kept.push_back(p);
  }
  list.swap(kept);
  return copies;
}

}  // namespace kp
