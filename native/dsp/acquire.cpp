// fpvDROID decoder: acquisition. See acquire.h.
#include "acquire.h"

#include <algorithm>
#include <cmath>

#include "fold.h"
#include "log.h"

namespace kp {

std::optional<PulseScore> score_pulses(const std::vector<float> &buf, size_t cnt, double fs, float sgn, float thr) {
  std::vector<double> edges;
  bool lo = false;
  size_t ls = 0;
  for (size_t i = 0; i < cnt; i++) {
    const float v = sgn * buf[i];
    if (!lo && v < thr) {
      lo = true;
      ls = i;
    } else if (lo && v >= thr) {
      lo = false;
      const double w = (i - ls) / fs * 1e6;
      if (w > 3.0 && w < 6.5) edges.push_back(static_cast<double>(ls));
    }
  }
  if (edges.size() < 30) return std::nullopt;
  std::vector<double> good;
  for (size_t k = 1; k < edges.size(); k++) {
    const double iv = (edges[k] - edges[k - 1]) / fs * 1e6;
    if (iv > 62.5 && iv < 65.0) good.push_back(iv);
  }
  PulseScore s{edges.size(), static_cast<float>(good.size()) / (edges.size() - 1), 0};
  if (!good.empty()) {
    std::sort(good.begin(), good.end());
    s.T_us = good[good.size() / 2];
  }
  return s;
}

std::optional<FoldLock> acquire_by_fold(const std::vector<float> &buf, size_t cnt, double fs, float sign, int std_force,
                                        const VideoStd *std_now, std::optional<double> T_known) {
  constexpr int kParts = kAcqParts, kNeed = 5;
  const double us = 1e-6 * fs;
  const int step = std::max(1, static_cast<int>(std::lround(0.2 * us)));  // every 0.2 us will do: 1 MHz of signal
  const size_t part = cnt / kParts;
  std::optional<FoldLock> got;
  Fold fold;
  for (const VideoStd *cand : {&kNtsc, &kPal}) {
    if ((std_force == 1 && cand != &kNtsc) || (std_force == 2 && cand != &kPal)) continue;
    const double T0 = T_known && cand == std_now ? *T_known : cand->T_us * us;
    const int K = static_cast<int>(T0 / step);
    fold.shape(static_cast<size_t>(std::max(K, 0)), fs, step, true);
    if (K < fold.bins(40.0) || part < static_cast<size_t>(32 * T0)) continue;
    double pos[kParts];
    float tips[kParts], blanks[kParts];
    bool ok[kParts];
    for (int p = 0; p < kParts; p++) {
      fold.v.assign(static_cast<size_t>(K), 0.f);
      fold.n.assign(static_cast<size_t>(K), 0);
      double ph = std::fmod(static_cast<double>(p * part), T0);
      for (size_t i = p * part; i < (p + 1) * part; i += static_cast<size_t>(step)) {
        const int k = std::min(K - 1, static_cast<int>(ph / step));
        fold.v[static_cast<size_t>(k)] += sign * buf[i];
        fold.n[static_cast<size_t>(k)]++;
        ph += step;
        if (ph >= T0) ph -= T0;
      }
      const std::optional<FoldPulse> hs = fold.pulse(0, K - 1);
      ok[p] = hs && hs->lowest_there();
      if (!ok[p]) continue;
      pos[p] = hs->edge * step;
      tips[p] = hs->tip;
      blanks[p] = hs->blank;
    }
    // the pulse's place from part to part, unwrapped, and the straight line through it
    double u[kParts], last = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    int n_ok = 0, prev = -1;
    for (int p = 0; p < kParts; p++) {
      if (!ok[p]) continue;
      last = prev < 0 ? pos[p] : last + std::remainder(pos[p] - pos[prev], T0);
      u[p] = last;
      prev = p;
      n_ok++;
      sx += p, sy += last, sxx += static_cast<double>(p) * p, sxy += p * last;
    }
    KP_LOGD("video acq by fold: %s: %d of %d parts show a pulse", cand->name, n_ok, kParts);
    if (n_ok < kNeed) continue;
    const double den = n_ok * sxx - sx * sx;
    const double slope = den > 0 ? (n_ok * sxy - sx * sy) / den : 0, at0 = (sy - slope * sx) / n_ok;
    int in = 0;
    float tip_m = 0, blank_m = 0;
    for (int p = 0; p < kParts; p++) {
      if (!ok[p] || std::fabs(u[p] - (at0 + slope * p)) > 0.5 * us) continue;
      in++;
      tip_m += tips[p];
      blank_m += blanks[p];
    }
    if (in < kNeed || in <= (got ? got->parts : 0) || std::fabs(slope) > 1.5 * us) continue;
    // each part's place is its middle's: the line through them, on to the buffer's end
    const double at_end = at0 + slope * (kParts - 0.5);
    got = FoldLock{cand, in, T0 + slope * T0 / static_cast<double>(part),
                   at_end + T0 * std::floor((static_cast<double>(cnt) - at_end) / T0),  // the last hsync in the buffer
                   tip_m / static_cast<float>(in), blank_m / static_cast<float>(in)};
  }
  return got;
}

}  // namespace kp
