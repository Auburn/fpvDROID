#include "wifi.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace kp {

namespace {
// ---------------------------------------------------------------------------------------- one step (measure_bursts)
// A slice more than 6 dB over the floor is on (upstream's survey). The floor is StepGrid's: noise reads 0 dB on it, as
// on upstream's, against which its thresholds were measured.
constexpr float kOnOver = 4;
// A 20 MHz channel's OFDM: 52 subcarriers, 16.6 MHz.
constexpr double kOccupiedHz = 16.6e6;
// A channel is judged when kChannelPresent of its bins are in the step's span, on those of them that are not on all the
// time (kSteadyDuty: a spur's, a pilot's beside it), when those are at least kJudgedShare of it. Not one that holds
// kSteadyRunHz on all the time in one piece: it is under a transmitter that never stops. Analog video is such a one:
// every line has its sync pulse, so what lies from the sync tip to the blanking level (2 to 5 MHz) is on in every
// slice. kSteadyRunHz is more than a strong spur lights through the window's sidelobes.
constexpr double kChannelPresent = 0.85, kJudgedShare = 0.5;
constexpr float kSteadyDuty = 0.9f;
constexpr double kSteadyRunHz = 1.5e6;
// A full slice has kFullShare of those bins on at once, and evenly: their 10th percentile within kFlatDb of their 90th
// (room for the ripple of a multipath channel indoors). An empty one has no more than kEmptyShare of them on.
constexpr double kFullShare = 0.9, kEmptyShare = 0.1;
constexpr float kFlatDb = 15;
// A full slice is on in its frames (6.4 us) one by one too, not only over the 0.1 ms: each OFDM symbol carries every
// subcarrier, where FM is on one frequency at a time (a ramp across a line covers ~1 MHz a frame). A single FFT's
// power is far noisier than a slice's mean of 16 (exponential: noise is over 3 dB 13.5 % of the time, a network 10 dB
// up 82 %), so a frame is filled when kFrameFill of the bins are over kFrameOver (3 dB) in it, and of kCheckFrames
// frames spread over the slice all but one must be (a burst may start or end inside it).
constexpr float kFrameOver = 2, kFrameFill = 0.5f;
constexpr int kCheckFrames = 4;

struct ChRange {
  int first, last;
};
// Which channels are looked for: every region's (the US plan, a superset of the EU's, where 144 and 169-177 are not
// used), since a router only uses what its country allows and what is seen is what is there. 68-96 are no Wi-Fi
// channels anywhere (5340-5480 MHz), and 36-64 are mostly under the band the tab sweeps.
constexpr ChRange kWifiChannels[] = {{36, 64}, {100, 144}, {149, 177}};
constexpr ChRange kBlocks80[] = {{36, 48}, {52, 64}, {100, 112}, {116, 128}, {132, 144}, {149, 161}, {165, 177}};
constexpr ChRange kBlocks160[] = {{36, 64}, {100, 128}, {149, 177}};

template <size_t N>
int block_of(const ChRange (&blocks)[N], int ch) {
  for (size_t i = 0; i < N; i++)
    if (ch >= blocks[i].first && ch <= blocks[i].last) return static_cast<int>(i);
  return -1;
}

// Each bin's strongest slice and the share of slices it is on in, over the floor (the bins outside the span: 0).
struct BinStats {
  float peak = 0, duty = 0;
};
std::vector<BinStats> bin_stats(const StepGrid &g) {
  const float on = kOnOver * g.floor();
  std::vector<BinStats> st(StepGrid::kN);
  std::vector<int> lit(StepGrid::kN, 0);
  for (size_t s = 0; s < g.slices(); s++) {
    const float *cell = g.slice(s);
    for (int k = 0; k < StepGrid::kN; k++) {
      st[k].peak = std::max(st[k].peak, cell[k]);
      lit[k] += cell[k] > on;
    }
  }
  for (int k = 0; k < StepGrid::kN; k++) {
    if (!g.in_span(k)) st[k] = BinStats{};
    else st[k] = BinStats{st[k].peak / g.floor(), static_cast<float>(lit[k]) / static_cast<float>(g.slices())};
  }
  return st;
}

// The bins of a slice (cell) evenly lit: OFDM's subcarriers all carry the same power, where FM video's levels stand
// some 30 dB over what its leaps between them leave in the bins in between.
bool even(const float *cell, const std::vector<int> &bins) {
  std::vector<float> level;
  level.reserve(bins.size());
  for (int k : bins) level.push_back(cell[k]);
  const size_t q10 = level.size() / 10, q90 = level.size() * 9 / 10;
  std::nth_element(level.begin(), level.begin() + static_cast<long>(q90), level.end());
  const float p90 = level[q90];
  std::nth_element(level.begin(), level.begin() + static_cast<long>(q10), level.begin() + static_cast<long>(q90));
  return level[q10] * std::pow(10.f, kFlatDb / 10) >= p90;
}

// The bins lit in slice s's frames one by one, as a burst of OFDM lights them.
bool frames_filled(const StepGrid &g, size_t s, const std::vector<int> &bins) {
  const float over = kFrameOver * g.floor();
  int filled = 0;
  for (int i = 0; i < kCheckFrames; i++) {
    const float *p = g.frame(s * StepGrid::kSliceFrames + i * StepGrid::kSliceFrames / kCheckFrames);
    size_t lit = 0;
    for (int k : bins) lit += p[k] > over;
    filled += lit >= kFrameFill * static_cast<double>(bins.size());
  }
  return filled >= kCheckFrames - 1;
}

// Channel ch as the step's slices show it, when nearly all of it is in the step's span and not under a transmitter
// that never stops.
std::optional<BurstStep::Channel> judge_channel(const StepGrid &g, const std::vector<BinStats> &st, double centre_hz,
                                                int ch) {
  const double half = kOccupiedHz / 2, d = wifi_mhz(ch) * 1e6 - centre_hz;  // the channel's centre, from the step's
  const int expect = 2 * static_cast<int>(half / g.bin_hz()) + 1;
  std::vector<int> judged;
  int present = 0, run = 0, longest = 0, prev = -2;
  for (int k = 0; k < StepGrid::kN; k++) {
    if (!g.in_span(k) || std::fabs(g.offset_hz(k) - d) > half) continue;
    present++;
    const bool steady = st[k].duty >= kSteadyDuty;
    if (!steady) judged.push_back(k);
    run = steady && k == prev + 1 ? run + 1 : steady;  // the DC notch breaks a run
    longest = std::max(longest, run);
    prev = k;
  }
  if (present < kChannelPresent * expect || judged.size() < kJudgedShare * expect || longest * g.bin_hz() >= kSteadyRunHz)
    return std::nullopt;
  BurstStep::Channel c{ch, 0, 0, 0};
  const float on = kOnOver * g.floor();
  const double n = static_cast<double>(judged.size());
  for (size_t s = 0; s < g.slices(); s++) {
    const float *cell = g.slice(s);
    size_t lit = 0;
    for (int k : judged) lit += cell[k] > on;
    c.empty += lit <= kEmptyShare * n;
    // only the slices that get this far are looked at closer, and only until kMinSlices pass
    if (c.full < kMinSlices && lit >= kFullShare * n && even(cell, judged) && frames_filled(g, s, judged)) c.full++;
  }
  std::vector<float> peak;
  for (int k : judged) peak.push_back(10 * std::log10(st[k].peak));
  c.level_db = median_of(std::move(peak));
  return c;
}

// ---------------------------------------------------------------------------------------- the burst picture
// One bin of it (dB over the floor of the step that measured it).
struct BurstBin {
  double mhz = 0;
  float peak_db = 0;  // the strongest slice of the last passes (it fades)
  float busy = 0;     // the share of the time it has been on, over the last kBusyPasses passes
  float on = 0;       // ... in the last step that measured it (the most of the step's bins in it): a pilot's is ~1
};

// Each step of a sweep laid on the band's bins (TimedBins, as BandMap lays its levels): what each bin did over the
// last passes, a bin taking the mean of the step's bins nearest it. As in upstream's survey grid, the strongest slice
// fades kPeakFadeDb over kPasses passes. The share of time on is a sliding average over kBusyPasses passes, started
// from nothing, so that the beacons of one capture early on do not read as a network in use, while one on a quarter
// of the time does within three passes.
class BurstMap {
public:
  static constexpr int kPasses = 20;        // ~40 s of a sweep over 300 MHz
  static constexpr float kPeakFadeDb = 20;  // over kPasses passes: 1 dB a pass
  static constexpr int kBusyPasses = 10;

  BurstMap(double lo_hz, double hi_hz, double bin_hz)
      : _bins(lo_hz, hi_hz, bin_hz), _peak(static_cast<size_t>(_bins.size())), _busy(_peak.size()), _on(_peak.size()) {}
  void add(const BurstStep &s, int64_t now_ms);
  void forget(double lo_hz, double hi_hz, int64_t older_than_ms) { _bins.forget(lo_hz, hi_hz, older_than_ms); }
  // The bins measured in lo..hi, in order.
  std::vector<BurstBin> get(double lo_hz, double hi_hz) const;

private:
  TimedBins _bins;
  std::vector<float> _peak, _busy, _on;  // linear, 0..1 and 0..1, while a bin is written
};

void BurstMap::add(const BurstStep &s, int64_t now_ms) {
  const float decay = std::pow(10.f, -kPeakFadeDb / kPasses / 10);
  for (size_t i = 0; i < s.bins.size();) {
    const int j = _bins.nearest(s.bins[i].hz);
    float peak = 0, duty = 0, most = 0;
    int n = 0;
    for (; i < s.bins.size() && _bins.nearest(s.bins[i].hz) == j; i++, n++) {
      peak += s.bins[i].peak;
      duty += s.bins[i].duty;
      most = std::max(most, s.bins[i].duty);
    }
    if (j < 0 || j >= _bins.size()) continue;
    if (_bins.written(j) == 0) _peak[j] = _busy[j] = 0;  // never seen, or forgotten: from nothing
    _peak[j] = std::max(peak / n, _peak[j] * decay);
    _busy[j] += (duty / n - _busy[j]) / kBusyPasses;
    _on[j] = most;
    _bins.write(j, now_ms);
  }
}

std::vector<BurstBin> BurstMap::get(double lo_hz, double hi_hz) const {
  int j0, j1;
  _bins.inside(lo_hz, hi_hz, &j0, &j1);
  std::vector<BurstBin> out;
  for (int j = j0; j <= j1; j++)
    if (_bins.written(j) > 0)
      out.push_back(BurstBin{_bins.hz(j) / 1e6, 10 * std::log10(std::max(_peak[j], 1e-6f)), _busy[j], _on[j]});
  return out;
}

// ---------------------------------------------------------------------------------------- the networks
// A channel counts when, in a step, kMinSlices slices had its bins on together and kMinSlices had them off together,
// and its strongest slices reach kWifiOverNoise over the band's typical strongest slice (the 25th percentile of the
// burst picture's: nearly all of it is noise), never under kWifiPeakMin. Upstream measured the noise's strongest slices
// on the adapter (54 sweeps, 5480-5940 MHz): median 5.9 dB, 99 % under 9.3 dB, whole stretches at 7-8 dB; an AP on
// channel 100, 24-27 dB.
constexpr float kWifiPeakMin = 6, kWifiOverNoise = 7;
// Not a channel in a strong pilot's sidebands (upstream's PR 14). Measured on the adapter, an analog VTX at 35-39 dB
// very close by lights up in bursts 10 to 45 MHz either side of it, which read as Wi-Fi on four channels. Its reach
// beyond its width grows kSidebandMhzPerDb a dB over kSidebandFromDb (6 to 40 MHz: at 37 dB, 50 MHz from its centre),
// and only what is at least kSidebandBelowDb weaker than it is taken for its sidebands: a real network next to a strong
// pilot, or any next to a weak one, still shows. The pilots are the carriers not on a network.
constexpr float kSidebandFromDb = 10, kSidebandBelowDb = 10;
constexpr double kSidebandMhzPerDb = 1.5, kSidebandMinMhz = 6, kSidebandMaxMhz = 40;
// Not a weak channel with a much stronger one close by: its shoulders. Measured upstream, a block of 80 MHz at 33 dB
// falls from 24 to 13 dB over the next 30 MHz.
constexpr double kWifiShoulderMhz = 40;
constexpr float kWifiShoulderDb = 10;
// Not a product of the receiver's front end. Its third-order intermodulation of a pilot (on all the time, as a VTX is)
// and a network lands at 2 fp - fw: it bursts when the network does and is as flat as the network's OFDM, so nothing
// in a step's slices tells it from a network. Seen on the phone: a VTX on R3 (5732.7 MHz, 23 dB) and a busy network on
// 149 (5745 MHz) lit channel 144 (2 x 5732.7 - 5745 = 5720.4 MHz) at 17 dB, idle, 6 dB under the VTX; the VTX alone
// does not (its skirts reach only the top 1.3 MHz of 144). So a channel is taken for such a product when a network is
// at least kIntermodBelowDb stronger and, within kIntermodMhz of halfway between the two (where a pilot has to be for
// its product with the network to land on the channel), the burst picture has bins on all the time (kSteadyDuty) and
// as much stronger. Their strongest slices, not the band picture's level: the same measure as the channel's, where a
// VTX's few dwelling levels stand well over its averaged spectrum (and where a busy network beside it does not run into
// it). The margin is small because a front end that is not overloaded makes the product far weaker than either. Not a
// channel in the network's own 80 MHz block: it may be that network, wider.
constexpr double kIntermodMhz = 2;
constexpr float kIntermodBelowDb = 3;
// In use: its bins on kWifiTrafficShare of the time or more (BurstBin::busy). Beacons take ~0.4 ms every 102.4 ms:
// under 2 % even for an AP that announces four networks.
constexpr float kWifiTrafficShare = 0.05f;
// A network seen within kWifiRecentMs is recent: a new one is weighed against those too.
constexpr int64_t kWifiRecentMs = 10000;

// Is w in pilot p's sidebands: within its reach, and much weaker?
bool sideband_of(const Wifi &w, const Carrier &p) {
  const double reach = std::clamp((p.snr_db - kSidebandFromDb) * kSidebandMhzPerDb, kSidebandMinMhz, kSidebandMaxMhz);
  return std::fabs(w.mhz - p.mhz) < p.width_mhz / 2 + reach && w.peak_db + kSidebandBelowDb <= p.snr_db;
}

// Is weak the shoulder of strong: another channel close by, much stronger?
bool shoulder_of(const Wifi &weak, const Wifi &strong) {
  return strong.ch != weak.ch && std::fabs(strong.mhz - weak.mhz) <= kWifiShoulderMhz &&
         strong.peak_db >= weak.peak_db + kWifiShoulderDb;
}

// Is w the front end's product of network net and a pilot halfway between them in the burst picture (bins, in order)?
bool intermod_of(const Wifi &w, const Wifi &net, const std::vector<BurstBin> &bins) {
  if (net.ch == w.ch || block_of(kBlocks80, net.ch) == block_of(kBlocks80, w.ch) ||
      w.peak_db + kIntermodBelowDb > net.peak_db)
    return false;
  const double mid = (w.mhz + net.mhz) / 2;
  auto it = std::lower_bound(bins.begin(), bins.end(), mid - kIntermodMhz, [](const BurstBin &b, double mhz) { return b.mhz < mhz; });
  for (; it != bins.end() && it->mhz <= mid + kIntermodMhz; ++it)
    if (it->on >= kSteadyDuty && it->peak_db >= w.peak_db + kIntermodBelowDb) return true;
  return false;
}

// The channels one sweep step shows lit as a network would light them (step: measure_bursts), against the burst
// picture over the span swept (bins), each in use or not.
std::vector<Wifi> candidates(const BurstStep &step, const std::vector<BurstBin> &bins) {
  std::vector<Wifi> seen;
  float lit_at = kWifiPeakMin;  // what a network's strongest slices have to reach
  if (!bins.empty()) {
    std::vector<float> pk;
    pk.reserve(bins.size());
    for (const auto &b : bins) pk.push_back(b.peak_db);
    std::nth_element(pk.begin(), pk.begin() + static_cast<long>(pk.size() / 4), pk.end());
    lit_at = std::max(kWifiPeakMin, pk[pk.size() / 4] + kWifiOverNoise);
  }
  const double half = kOccupiedHz / 2e6;
  for (const auto &c : step.channels) {
    if (c.full < kMinSlices || c.empty < kMinSlices || c.level_db < lit_at) continue;
    Wifi w;
    w.ch = c.ch;
    w.mhz = wifi_mhz(c.ch);
    w.peak_db = c.level_db;
    std::vector<float> busy;
    auto it = std::lower_bound(bins.begin(), bins.end(), w.mhz - half, [](const BurstBin &b, double mhz) { return b.mhz < mhz; });
    for (; it != bins.end() && it->mhz <= w.mhz + half; ++it) busy.push_back(it->busy);
    w.traffic = median_of(std::move(busy)) >= kWifiTrafficShare;
    seen.push_back(w);
  }
  return seen;
}

// The pilots among what the band picture has on the air (carriers: find_carriers): the carriers not centred on a
// network of seen or recent, which are those networks.
std::vector<Carrier> pilots_of(const std::vector<Carrier> &carriers, const std::vector<Wifi> &seen,
                               const std::vector<Wifi> &recent) {
  std::vector<Carrier> out;
  for (const auto &c : carriers) {
    bool on = false;
    for (const auto *list : {&seen, &recent})
      for (const auto &w : *list) on = on || std::fabs(c.mhz - w.mhz) <= kOccupiedHz / 2e6;
    if (!on) out.push_back(c);
  }
  return out;
}

// Of the channels a step has seen lit, the networks: not a pilot's sidebands, nor the shoulder of another network or
// the product of one and a pilot (another of seen, or one of recent: the networks seen in the last few seconds; bins:
// the burst picture).
std::vector<Wifi> networks_of(const std::vector<Wifi> &seen, const std::vector<Wifi> &recent,
                              const std::vector<Carrier> &pilots, const std::vector<BurstBin> &bins) {
  std::vector<Wifi> out;
  for (const auto &w : seen) {
    bool not_one = false;
    for (const auto &p : pilots) not_one = not_one || sideband_of(w, p);
    for (const auto *list : {&seen, &recent})
      for (const auto &o : *list) not_one = not_one || shoulder_of(w, o) || intermod_of(w, o, bins);
    if (!not_one) out.push_back(w);
  }
  return out;
}

// The networks seen, each kept kWifiHoldMs from the last time. A network once seen in use stays so while it is kept.
class WifiMemory {
public:
  // What a step showed at t_ms, with the burst picture it was weighed against. A network kept that is now the shoulder
  // of one of these, or its product with a pilot, is forgotten: a step lower in the band may have seen the shoulder or
  // the product before the step of the network itself.
  void update(const std::vector<Wifi> &now, const std::vector<BurstBin> &bins, int64_t t_ms);
  std::vector<Wifi> known(int64_t t_ms, int64_t within_ms = kWifiHoldMs) const;  // by channel

private:
  struct Entry {
    Wifi w;
    int64_t last_ms = 0;
  };
  std::vector<Entry> _nets;
};

void WifiMemory::update(const std::vector<Wifi> &now, const std::vector<BurstBin> &bins, int64_t t_ms) {
  _nets.erase(std::remove_if(_nets.begin(), _nets.end(),
                             [&](const Entry &e) {
                               if (t_ms - e.last_ms > kWifiHoldMs) return true;
                               for (const auto &w : now)
                                 if (shoulder_of(e.w, w) || intermod_of(e.w, w, bins)) return true;
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
}  // namespace

BurstStep measure_bursts(const StepGrid &g, double centre_hz, double reach_hz) {
  BurstStep out;
  if (g.slices() < 8 || !(g.floor() > 0)) return out;
  const std::vector<BinStats> st = bin_stats(g);
  for (int k = 0; k < StepGrid::kN; k++) {
    const double f = g.offset_hz(k);
    if (g.in_span(k) && f >= -reach_hz && f < reach_hz) out.bins.push_back({centre_hz + f, st[k].peak, st[k].duty});
  }
  for (const auto &r : kWifiChannels)
    for (int ch = r.first; ch <= r.last; ch += 4)
      if (std::fabs(wifi_mhz(ch) * 1e6 - centre_hz) < g.fs() / 2)
        if (const auto c = judge_channel(g, st, centre_hz, ch)) out.channels.push_back(*c);
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

const WifiNet *wifi_for(const std::vector<WifiNet> &nets, const Carrier &c) {
  for (const auto &n : nets)
    if (c.mhz >= n.lo_mhz && c.mhz <= n.hi_mhz && c.width_mhz <= n.hi_mhz - n.lo_mhz) return &n;
  return nullptr;
}

struct WifiWatch::State {
  BurstMap map;
  WifiMemory memory;
  int64_t last_ms = 0;  // the last step's time
};

WifiWatch::WifiWatch(double lo_hz, double hi_hz, double bin_hz)
    : _s(new State{BurstMap(lo_hz, hi_hz, bin_hz), WifiMemory(), 0}) {}

WifiWatch::~WifiWatch() = default;

void WifiWatch::forget(double lo_hz, double hi_hz, int64_t older_than_ms) { _s->map.forget(lo_hz, hi_hz, older_than_ms); }

void WifiWatch::add(const BurstStep &s, double lo_hz, double hi_hz, const std::vector<Carrier> &carriers,
                    int64_t now_ms) {
  _s->last_ms = now_ms;
  _s->map.add(s, now_ms);
  const std::vector<BurstBin> bins = _s->map.get(lo_hz, hi_hz);
  const std::vector<Wifi> seen = candidates(s, bins);
  const std::vector<Wifi> recent = _s->memory.known(now_ms, kWifiRecentMs);
  _s->memory.update(networks_of(seen, recent, pilots_of(carriers, seen, recent), bins), bins, now_ms);
}

std::vector<WifiNet> WifiWatch::networks(double lo_hz, double hi_hz) const {
  std::vector<WifiNet> out;
  for (const auto &n : group_wifi(_s->memory.known(_s->last_ms)))
    if (n.hi_mhz > lo_hz / 1e6 && n.lo_mhz < hi_hz / 1e6) out.push_back(n);
  return out;
}

}  // namespace kp
