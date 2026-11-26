#include "wifi.h"

#include <algorithm>
#include <cmath>

namespace kp {

namespace {
constexpr int kN = 256, kFrames = 16, kSlice = kN * kFrames;  // 0.1 ms slices at 40 MS/s
constexpr double kDcHz = 0.75e6;                               // the DC notch, as BandMap leaves out
// The floor is a low percentile of all of a capture's cells (slice x bin), not of the bins' means: a busy network as
// wide as the step (80 MHz) raises every bin's mean, but between its frames the cells are noise. Each cell of noise is
// the mean of 16 FFT powers (Gamma(16, 1/16)), whose 10th percentile is 0.696 of its mean: divided by that, noise
// reads 0 dB, as on upstream's floor (the 10th percentile of its bins' means), against which its thresholds were
// measured.
constexpr double kFloorQuantile = 0.10, kNoiseP10 = 0.696;
constexpr float kOnOver = 4;  // a slice more than 6 dB over the floor is on (upstream's survey)
// A single FFT's power is far noisier than a slice's mean of 16 (exponential: noise is over 3 dB 13.5 % of the time,
// a network 10 dB up 82 %): a frame is filled when half of the channel's bins are over 3 dB in it, and a slice passes
// with all but one of the frames looked at filled (a burst may start or end inside it).
constexpr float kFrameOver = 2, kFrameFill = 0.5f;
constexpr int kCheckFrames = 4;

float median(std::vector<float> v) {
  if (v.empty()) return 0;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

struct ChRange {
  int first, last;
};
constexpr ChRange kWifiChannels[] = {{36, 64}, {100, 144}, {149, 177}};
constexpr ChRange kBlocks80[] = {{36, 48}, {52, 64}, {100, 112}, {116, 128}, {132, 144}, {149, 161}, {165, 177}};
constexpr ChRange kBlocks160[] = {{36, 64}, {100, 128}, {149, 177}};

template <size_t N>
int block_of(const ChRange (&blocks)[N], int ch) {
  for (size_t i = 0; i < N; i++)
    if (ch >= blocks[i].first && ch <= blocks[i].last) return static_cast<int>(i);
  return -1;
}

double sideband_reach(const Carrier &c) {
  return std::clamp((c.snr_db - kSidebandFromDb) * kSidebandMhzPerDb, kSidebandMinMhz, kSidebandMaxMhz);
}
}  // namespace

BurstStep measure_bursts(const cf *x, size_t n, double fs, double centre_hz, double reach_hz, double span_hz) {
  BurstStep out;
  out.centre_hz = centre_hz;
  const size_t slices = n / kSlice;
  if (slices < 8 || fs <= 0) return out;
  const double bin = fs / kN;
  // the FFT bins kept, from the lowest frequency up, and each one's offset from the centre (in bins)
  std::vector<int> use, off;
  for (int o = -kN / 2; o < kN / 2; o++) {
    if (std::fabs(o * bin) > span_hz || std::fabs(o * bin) < kDcHz) continue;
    use.push_back((o + kN) % kN);
    off.push_back(o);
  }
  if (use.empty()) return out;
  const size_t nu = use.size();
  float win[kN];
  for (int i = 0; i < kN; i++) win[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * M_PI * i / kN));
  cf buf[kN];
  // the FFT of frame fr of slice s, into buf
  const auto frame = [&](size_t s, int fr) {
    const cf *src = x + s * kSlice + static_cast<size_t>(fr) * kN;
    for (int i = 0; i < kN; i++) buf[i] = src[i] * win[i];
    fft_inplace(buf, kN);
  };
  std::vector<float> cell(slices * nu);  // the power of each slice in each bin kept, slice by slice
  float acc[kN];
  for (size_t s = 0; s < slices; s++) {
    std::fill(acc, acc + kN, 0.f);
    for (int fr = 0; fr < kFrames; fr++) {
      frame(s, fr);
      for (int k = 0; k < kN; k++) acc[k] += std::norm(buf[k]);
    }
    for (size_t u = 0; u < nu; u++) cell[s * nu + u] = acc[use[u]] / kFrames;
  }
  std::vector<float> sorted = cell;
  const size_t q = static_cast<size_t>(kFloorQuantile * static_cast<double>(sorted.size()));
  std::nth_element(sorted.begin(), sorted.begin() + q, sorted.end());
  const float floor = static_cast<float>(sorted[q] / kNoiseP10);
  if (!(floor > 0)) return out;  // no signal at all (a stream of zeros)
  const float on = kOnOver * floor;
  std::vector<float> peak(nu), duty(nu);
  for (size_t u = 0; u < nu; u++) {
    float pk = 0;
    int lit = 0;
    for (size_t s = 0; s < slices; s++) {
      const float v = cell[s * nu + u];
      pk = std::max(pk, v);
      lit += v > on;
    }
    peak[u] = pk / floor;
    duty[u] = static_cast<float>(lit) / static_cast<float>(slices);
  }
  const long grid = std::lround((BurstMap::kHi - BurstMap::kLo) / BurstMap::kBin) + 1;
  for (size_t u = 0; u < nu; u++) {
    const double f = off[u] * bin;
    const long i = std::lround((centre_hz + f - BurstMap::kLo) / BurstMap::kBin);
    if (f >= -reach_hz && f < reach_hz && i >= 0 && i < grid) out.bins.push_back({i, peak[u], duty[u]});
  }
  // the channels: their bins in this capture, those not on all the time judged slice by slice
  const double half = kWifiOccupiedMhz / 2 * 1e6;
  const int expect = 2 * static_cast<int>(half / bin) + 1;
  const float flat = std::pow(10.f, kFlatDb / 10);
  std::vector<size_t> judged;
  std::vector<float> level;
  for (const auto &g : kWifiChannels)
    for (int ch = g.first; ch <= g.last; ch += 4) {
      const double d = wifi_mhz(ch) * 1e6 - centre_hz;  // the channel's centre, from the capture's
      if (std::fabs(d) > span_hz + half) continue;
      int present = 0, run = 0, longest = 0;
      judged.clear();
      for (size_t u = 0; u < nu; u++) {
        if (std::fabs(off[u] * bin - d) > half) continue;
        present++;
        const bool steady = duty[u] >= kSteadyDuty;
        if (!steady) judged.push_back(u);
        run = steady && u > 0 && off[u] == off[u - 1] + 1 ? run + 1 : steady;  // the DC notch breaks a run
        longest = std::max(longest, run);
      }
      if (present < kChannelPresent * expect || judged.size() < kJudgedShare * expect ||
          longest * bin >= kSteadyRunMhz * 1e6)
        continue;
      int full = 0, empty = 0;
      for (size_t s = 0; s < slices; s++) {
        size_t lit = 0;
        for (size_t u : judged) lit += cell[s * nu + u] > on;
        empty += lit <= kEmptyShare * static_cast<double>(judged.size());
        if (lit < kFullShare * static_cast<double>(judged.size()) || full >= kMinSlices) continue;
        // on together, and evenly: OFDM's subcarriers all carry the same power, where FM video's levels stand some
        // 30 dB over what its leaps between them leave in the bins in between
        level.clear();
        for (size_t u : judged) level.push_back(cell[s * nu + u]);
        const size_t q10 = level.size() / 10, q90 = level.size() * 9 / 10;
        std::nth_element(level.begin(), level.begin() + q90, level.end());
        const float p90 = level[q90];
        std::nth_element(level.begin(), level.begin() + q10, level.begin() + q90);
        if (level[q10] * flat < p90) continue;
        // and in its frames (6.4 us) one by one, not only over the 0.1 ms: each OFDM symbol carries all the
        // subcarriers, where FM is on one frequency at a time (a ramp across a line covers ~1 MHz a frame). Only the
        // slices that got this far are taken again, kCheckFrames frames of each, and only until kMinSlices pass.
        int filled = 0;
        for (int i = 0; i < kCheckFrames; i++) {
          frame(s, i * kFrames / kCheckFrames);
          size_t lit_fr = 0;
          for (size_t u : judged) lit_fr += std::norm(buf[use[u]]) > kFrameOver * floor;
          filled += lit_fr >= kFrameFill * static_cast<double>(judged.size());
        }
        full += filled >= kCheckFrames - 1;
      }
      std::vector<float> pk;
      for (size_t u : judged) pk.push_back(10 * std::log10(peak[u]));
      out.channels.push_back({ch, full, empty, median(pk)});
    }
  return out;
}

BurstMap::BurstMap() : _acc(static_cast<size_t>(std::lround((kHi - kLo) / kBin) + 1)) {}

void BurstMap::add(const BurstStep &s, int64_t now_ms) {
  const float decay = std::pow(10.f, -kPeakFadeDb / kPasses / 10);
  const int64_t t = std::max<int64_t>(now_ms, 1);  // 0 marks a bin never measured
  for (const auto &b : s.bins) {
    if (b.index < 0 || b.index >= static_cast<long>(_acc.size())) continue;
    Acc &a = _acc[static_cast<size_t>(b.index)];
    a.peak = std::max(b.peak, a.peak * decay);
    a.busy += (b.duty - a.busy) / kBusyPasses;
    a.t = t;
  }
}

void BurstMap::forget(double lo_hz, double hi_hz, int64_t older_than_ms) {
  const long m = static_cast<long>(_acc.size());
  const long j0 = std::max(0L, static_cast<long>(std::floor((lo_hz - kLo) / kBin)));
  const long j1 = std::min(m - 1, static_cast<long>(std::ceil((hi_hz - kLo) / kBin)));
  for (long j = j0; j <= j1; j++) {
    Acc &a = _acc[static_cast<size_t>(j)];
    if (a.t > 0 && a.t < older_than_ms) a = Acc{};
  }
}

std::vector<BurstBin> BurstMap::get(double lo_hz, double hi_hz) const {
  const long m = static_cast<long>(_acc.size());
  const long j0 = std::max(0L, static_cast<long>(std::ceil((lo_hz - kLo) / kBin)));
  const long j1 = std::min(m - 1, static_cast<long>(std::floor((hi_hz - kLo) / kBin)));
  std::vector<BurstBin> out;
  for (long j = j0; j <= j1; j++) {
    const Acc &a = _acc[static_cast<size_t>(j)];
    if (a.t == 0) continue;
    out.push_back(BurstBin{(kLo + static_cast<double>(j) * kBin) / 1e6, 10 * std::log10(std::max(a.peak, 1e-6f)), a.busy});
  }
  return out;
}

std::vector<Wifi> find_wifi(const BurstStep &step, const std::vector<BurstBin> &bins, const std::vector<Wifi> &recent,
                            const std::vector<Carrier> &carriers) {
  std::vector<Wifi> seen;
  float lit_at = kWifiPeakMin;  // what a network's strongest slices have to reach
  if (!bins.empty()) {
    std::vector<float> pk;
    pk.reserve(bins.size());
    for (const auto &b : bins) pk.push_back(b.peak_db);
    std::nth_element(pk.begin(), pk.begin() + pk.size() / 4, pk.end());
    lit_at = std::max(kWifiPeakMin, pk[pk.size() / 4] + kWifiOverNoise);
  }
  const double half = kWifiOccupiedMhz / 2;
  for (const auto &c : step.channels) {
    if (c.full < kMinSlices || c.empty < kMinSlices || c.level_db < lit_at) continue;
    Wifi w;
    w.ch = c.ch;
    w.mhz = wifi_mhz(c.ch);
    w.peak_db = c.level_db;
    std::vector<float> busy;
    auto it = std::lower_bound(bins.begin(), bins.end(), w.mhz - half, [](const BurstBin &b, double mhz) { return b.mhz < mhz; });
    for (; it != bins.end() && it->mhz <= w.mhz + half; ++it) busy.push_back(it->busy);
    w.traffic = median(busy) >= kWifiTrafficShare;
    seen.push_back(w);
  }
  const std::vector<Wifi> *lists[] = {&seen, &recent};
  // a carrier centred on a network (this step's or a recent one) is that network, not a pilot
  const auto on_network = [&](double mhz) {
    for (const auto *list : lists)
      for (const auto &w : *list)
        if (std::fabs(mhz - w.mhz) <= half) return true;
    return false;
  };
  std::vector<Wifi> out;
  for (const auto &w : seen) {
    bool sideband = false;
    for (const auto &p : carriers) {
      if (on_network(p.mhz)) continue;
      const double d = std::fabs(w.mhz - p.mhz);
      sideband = sideband || (d < p.width_mhz / 2 + sideband_reach(p) && w.peak_db + kSidebandBelowDb <= p.snr_db);
    }
    bool shoulder = false;
    for (const auto *list : lists)
      for (const auto &o : *list)
        shoulder = shoulder || (o.ch != w.ch && std::fabs(o.mhz - w.mhz) <= kWifiShoulderMhz &&
                                o.peak_db >= w.peak_db + kWifiShoulderDb);
    if (!sideband && !shoulder) out.push_back(w);
  }
  return out;
}

void WifiMemory::update(const std::vector<Wifi> &now, int64_t t_ms) {
  _nets.erase(std::remove_if(_nets.begin(), _nets.end(),
                             [&](const Entry &e) {
                               if (t_ms - e.last_ms > kWifiHoldMs) return true;
                               for (const auto &w : now)
                                 if (w.ch != e.w.ch && std::fabs(w.mhz - e.w.mhz) <= kWifiShoulderMhz &&
                                     w.peak_db >= e.w.peak_db + kWifiShoulderDb)
                                   return true;
                               return false;
                             }),
              _nets.end());
  for (const auto &w : now) {
    auto it = std::find_if(_nets.begin(), _nets.end(), [&](const Entry &e) { return e.w.ch == w.ch; });
    if (it == _nets.end()) {
      _nets.push_back(Entry{w, t_ms});
      continue;
    }
    it->w.peak_db = std::max(it->w.peak_db, w.peak_db);
    it->w.traffic = it->w.traffic || w.traffic;
    it->last_ms = t_ms;
  }
}

std::vector<Wifi> WifiMemory::known(int64_t t_ms, int64_t within_ms) const {
  std::vector<Wifi> out;
  for (const auto &e : _nets)
    if (t_ms - e.last_ms <= within_ms) out.push_back(e.w);
  std::sort(out.begin(), out.end(), [](const Wifi &a, const Wifi &b) { return a.ch < b.ch; });
  return out;
}

std::string WifiNet::label() const {
  std::string s = "Wi-Fi " + std::to_string(first);
  if (last != first) s += "\xe2\x80\x93" + std::to_string(last);  // an en dash
  return s;
}

std::vector<WifiNet> group_wifi(const std::vector<Wifi> &wifi) {
  auto w = wifi;
  std::sort(w.begin(), w.end(), [](const Wifi &a, const Wifi &b) { return a.ch < b.ch; });
  std::vector<WifiNet> nets;
  for (const auto &x : w) {
    if (!nets.empty() && x.ch == nets.back().last + 4 && block_of(kBlocks80, x.ch) >= 0 &&
        block_of(kBlocks80, x.ch) == block_of(kBlocks80, nets.back().last)) {
      nets.back().last = x.ch;
      nets.back().busy = nets.back().busy || x.traffic;
      continue;
    }
    nets.push_back(WifiNet{x.ch, x.ch, 0, 0, x.traffic});
  }
  // two 80 MHz blocks lit whole, side by side in one 160 MHz block
  const auto whole80 = [](const WifiNet &n) {
    const int b = block_of(kBlocks80, n.first);
    return b >= 0 && n.first == kBlocks80[b].first && n.last == kBlocks80[b].last;
  };
  for (size_t i = 0; i + 1 < nets.size();) {
    const int b = block_of(kBlocks160, nets[i].first);
    if (b >= 0 && whole80(nets[i]) && whole80(nets[i + 1]) && nets[i + 1].first == nets[i].last + 4 &&
        b == block_of(kBlocks160, nets[i + 1].last)) {
      nets[i].last = nets[i + 1].last;
      nets[i].busy = nets[i].busy || nets[i + 1].busy;
      nets.erase(nets.begin() + static_cast<long>(i) + 1);
    } else {
      i++;
    }
  }
  for (auto &n : nets) {
    n.lo_mhz = wifi_mhz(n.first) - 10;
    n.hi_mhz = wifi_mhz(n.last) + 10;
  }
  return nets;
}

void WifiWatch::add(const BurstStep &s, double lo_hz, double hi_hz, const std::vector<Carrier> &carriers,
                    int64_t now_ms) {
  _map.add(s, now_ms);
  _memory.update(find_wifi(s, _map.get(lo_hz, hi_hz), _memory.known(now_ms, kWifiRecentMs), carriers), now_ms);
}

std::vector<WifiNet> WifiWatch::networks(double lo_hz, double hi_hz, int64_t now_ms) const {
  std::vector<WifiNet> out;
  for (const auto &n : group_wifi(_memory.known(now_ms)))
    if (n.hi_mhz > lo_hz / 1e6 && n.lo_mhz < hi_hz / 1e6) out.push_back(n);
  return out;
}

}  // namespace kp
