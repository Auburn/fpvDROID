// Ported from FPVDesktopWiFiReceiver's src/kp_ext/digital_scan.cpp (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver,
// GPL-2.0): the thresholds and the reasons for them are upstream's. What is fpvDROID's own: band_duty and tdd_frame
// read the step's grid (StepGrid: its FFTs taken once a step, for every measure), the timing is worked out only for
// blocks that may be digital, every rule is in judge, and the verdicts are kept from pass to pass (DigitalMap).
#include "digital.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "log.h"

namespace kp {

namespace {
constexpr float kAboveFloorDb = 6;     // the band a block takes: what stands over the floor + 6 dB (as find_humps)
constexpr float kMinSnrDb = 8;         // a block's median level over the floor
constexpr double kMinVisible = 6e6;    // a narrower piece is left out (and not joined with the next step's either)
constexpr double kJoinGap = 0.5e6;     // narrow gaps inside a block (a null subcarrier at DC, the chip's DC notch)
constexpr double kClipMargin = 0.3e6;  // this close to the edge of the span: cut
constexpr float kMinFlat = 0.8f;       // OFDM ~0.95; FM with a picture, less
constexpr float kMinFlatTdd = 0.3f;    // with DJI's frame (see judge)
constexpr float kMinDuty = 0.9f;       // HDZero sends without a break; Wi-Fi comes in bursts
constexpr double kMinWidth = 12e6, kMaxWidth = 31e6;
constexpr double kHdzWideMin = 22e6, kHdzNarrowMin = 14e6;  // 27 and 17 MHz as specified
constexpr double kHdzMaxOffMhz = 2;    // measured centre against the channel (the transmitter's and adapter's crystals)
// TDD frames (tdd_frame)
constexpr double kTddMinFrame = 1e-3, kTddMaxFrame = 4e-3;  // at least 3 frames in a step's 12 ms
constexpr double kTddOnMin = 0.15, kTddOnMax = 0.95;        // neither almost silent nor continuous (HDZero)
constexpr double kTddMinRepeat = 0.7;  // how much the pattern repeats a period later, chance taken out (0..1)
constexpr double kDjiFrame = 2.5e-3, kDjiFrameTol = 0.1e-3;
// the timing measures: band power in the grid's frames (6.4 us at 40 MS/s), summed over kDutyFrames for band_duty
// (~0.1 ms slices) and over kTddFrames for tdd_frame (12.8 us slices)
constexpr int kDutyFrames = StepGrid::kSliceFrames, kTddFrames = 2;

// The grid's bins in the band f0..f1 (relative to the centre) less 10 % at each side (where the edges fall): first
// and one past the last, in a row (the grid's bins are fftshifted). Empty when first == last.
struct Bins {
  int first = 0, end = 0;
};
Bins band_bins(const StepGrid &g, double f0_hz, double f1_hz) {
  const double w = f1_hz - f0_hz;
  const double a = f0_hz + 0.1 * w, b = f1_hz - 0.1 * w;
  Bins bins{StepGrid::kN, StepGrid::kN};
  for (int k = 0; k < StepGrid::kN; k++) {
    const double f = g.offset_hz(k);
    if (f < a || f > b) continue;
    if (bins.first == StepGrid::kN) bins.first = k;
    bins.end = k + 1;
  }
  return bins;
}

// The power of each band (its bins) in each of the grid's frames: out[b][i] is band b in frame i. band_duty and
// tdd_frame are both sums of these over a few frames (upstream worked out each with FFTs of its own, the same ones).
std::vector<std::vector<double>> frame_power(const StepGrid &g, const std::vector<Bins> &bands) {
  std::vector<std::vector<double>> out(bands.size(), std::vector<double>(g.frames(), 0.0));
  for (size_t f = 0; f < g.frames(); f++) {
    const float *p = g.frame(f);
    for (size_t b = 0; b < bands.size(); b++) {
      double sum = 0;
      for (int k = bands[b].first; k < bands[b].end; k++) sum += p[k];
      out[b][f] = sum;
    }
  }
  return out;
}

// band_duty from a band's frame powers: slices of kDutyFrames, a slice there when within 6 dB of the 95th percentile
// of the slices.
float duty_of(const std::vector<double> &frames) {
  const size_t slices = frames.size() / kDutyFrames;
  if (slices < 8) return 0;
  std::vector<double> p(slices, 0);
  for (size_t s = 0; s < slices; s++)
    for (int f = 0; f < kDutyFrames; f++) p[s] += frames[s * kDutyFrames + f];
  std::vector<double> sorted = p;
  std::sort(sorted.begin(), sorted.end());
  const double ref = sorted[std::min(sorted.size() - 1, sorted.size() * 95 / 100)];
  size_t on = 0;
  for (double v : p) on += v >= ref / 4;
  return static_cast<float>(on) / static_cast<float>(slices);
}

// tdd_frame from a band's frame powers, in slices of kTddFrames.
double frame_of(const std::vector<double> &frames, double fs) {
  const size_t slices = frames.size() / kTddFrames;
  const double slice_s = StepGrid::kN * kTddFrames / fs;
  if (slices * slice_s < 3 * kTddMinFrame) return 0;
  std::vector<float> p(slices);
  for (size_t s = 0; s < slices; s++) {
    double acc = 0;
    for (int f = 0; f < kTddFrames; f++) acc += frames[s * kTddFrames + f];
    p[s] = static_cast<float>(10 * std::log10(acc + 1e-9));
  }
  // sending or quiet: halfway (in dB) between the noise (5th percentile: the gaps) and the level (90th)
  std::vector<float> sorted = p;
  std::sort(sorted.begin(), sorted.end());
  const float lo = sorted[slices * 5 / 100], hi = sorted[slices * 90 / 100];
  if (hi - lo < kMinSnrDb) return 0;
  const float thr = (lo + hi) / 2;
  std::vector<char> up(slices);
  for (size_t i = 0; i < slices; i++) up[i] = p[i] > thr;
  // At a low SNR a slice of noise falls under the threshold in the middle of a burst: gaps of under 3 slices (~38 us;
  // the frame's own are 80 us or more) are filled in.
  for (size_t i = 1; i < slices;) {
    if (up[i]) {
      i++;
      continue;
    }
    size_t j = i;
    while (j < slices && !up[j]) j++;
    if (j < slices && up[i - 1] && j - i < 3)
      for (size_t k = i; k < j; k++) up[k] = 1;
    i = j;
  }
  size_t on = 0;
  for (char u : up) on += u;
  const double q = static_cast<double>(on) / static_cast<double>(slices);
  if (q < kTddOnMin || q > kTddOnMax) return 0;
  // The send/quiet pattern repeats a period later, whatever the frame is like (DJI with video: 1.73 + 0.37 ms
  // sending; with no goggles: 0.41 + 0.43 ms and a gap of 1.53 ms). repeat: the matches at a lag, less those chance
  // would give (q^2 + (1-q)^2): 1 = the same, 0 = as Wi-Fi, no rhythm.
  const double chance = q * q + (1 - q) * (1 - q);
  const size_t lag_lo = static_cast<size_t>(std::ceil(kTddMinFrame / slice_s));
  const size_t lag_hi = std::min(slices / 2, static_cast<size_t>(kTddMaxFrame / slice_s));
  std::vector<double> repeat(lag_hi + 1, -1);
  double best = -1;
  for (size_t lag = lag_lo; lag <= lag_hi; lag++) {
    size_t same = 0;
    for (size_t i = 0; i + lag < slices; i++) same += up[i] == up[i + lag];
    const double m = static_cast<double>(same) / static_cast<double>(slices - lag);
    repeat[lag] = (m - chance) / (1 - chance);
    best = std::max(best, repeat[lag]);
  }
  if (best < kTddMinRepeat) return 0;
  // the period: the first peak almost as good as the best (2 and 3 periods match too)
  for (size_t lag = lag_lo; lag <= lag_hi; lag++) {
    if (repeat[lag] < best - 0.05) continue;
    while (lag < lag_hi && repeat[lag + 1] > repeat[lag]) lag++;
    return static_cast<double>(lag) * slice_s;
  }
  return 0;
}

// A block whose timing is worth working out (upstream's DigitalScan::wants_timing): strong enough, flat enough for
// DJI at least, and either cut (its other part is in the next step) or wide enough. Anything else is never judged
// digital by itself (judge), so it is not timed; as one half of a signal cut between two steps, the other half's
// timing is the one taken (upstream took the better of the two halves' figures, measured for any block).
bool may_be_digital(const WideRegion &r) {
  const double w = r.f1_hz - r.f0_hz;
  const bool cut = r.clip_lo || r.clip_hi;
  return r.snr_db >= kMinSnrDb && r.flat >= kMinFlatTdd && w <= kMaxWidth && (cut || w >= kMinWidth);
}

bool is_dji_frame(double frame_s) { return std::fabs(frame_s - kDjiFrame) <= kDjiFrameTol; }

// HDZero's channels: Raceband, E1, F1, F2 and F4, and its unlockable L1-L8 (the analog low band's), as the app's
// Plans.hdzero has them.
bool on_hdzero_channel(double hz) {
  static const std::vector<double> mhz = [] {
    std::vector<double> v;
    for (const auto &c : fpv_channels()) {
      const std::string n = c.name;
      if (n[0] == 'R' || n[0] == 'L' || n == "E1" || n == "F1" || n == "F2" || n == "F4") v.push_back(c.mhz);
    }
    return v;
  }();
  for (double c : mhz)
    if (std::fabs(hz / 1e6 - c) <= kHdzMaxOffMhz) return true;
  return false;
}
}  // namespace

std::vector<WideRegion> find_wide_regions(const std::vector<float> &sm, double fs, double edge_hz) {
  std::vector<WideRegion> out;
  const int n = static_cast<int>(sm.size());
  if (n < 64) return out;
  const double bin = fs / n;
  auto freq = [&](int i) { return (i - n / 2) * bin; };
  // only inside the span used: past it the chip's filter rolls off and would pull the floor down
  int lo = n, hi = -1;
  for (int i = 0; i < n; i++)
    if (std::fabs(freq(i)) < edge_hz) {
      lo = std::min(lo, i);
      hi = std::max(hi, i);
    }
  if (hi - lo < 16) return out;
  // the floor: the 10th percentile of the span (a 27 MHz signal takes ~80 % of its 33 MHz)
  std::vector<float> pb(sm.begin() + lo, sm.begin() + hi + 1);
  std::nth_element(pb.begin(), pb.begin() + pb.size() / 10, pb.end());
  const float floor = pb[pb.size() / 10];
  const float thr = floor + kAboveFloorDb;

  // runs over the threshold, those apart by narrow gaps joined
  std::vector<std::pair<int, int>> runs;  // [i, j)
  for (int i = lo; i <= hi;) {
    if (sm[i] <= thr) {
      i++;
      continue;
    }
    int j = i;
    while (j <= hi && sm[j] > thr) j++;
    if (!runs.empty() && (i - runs.back().second) * bin <= kJoinGap) runs.back().second = j;
    else runs.emplace_back(i, j);
    i = j;
  }
  for (const auto &[i, j] : runs) {
    WideRegion r;
    r.f0_hz = freq(i) - bin / 2;
    r.f1_hz = freq(j - 1) + bin / 2;
    if (r.f1_hz - r.f0_hz < kMinVisible) continue;
    r.clip_lo = r.f0_hz <= -edge_hz + kClipMargin;
    r.clip_hi = r.f1_hz >= edge_hz - kClipMargin;
    const std::vector<float> in(sm.begin() + i, sm.begin() + j);
    const float top = median_of(in);
    r.snr_db = top - floor;
    int flat = 0;
    for (float v : in) flat += std::fabs(v - top) <= 3.f;
    r.flat = static_cast<float>(flat) / static_cast<float>(in.size());
    out.push_back(r);
  }
  return out;
}

float band_duty(const StepGrid &g, double f0_hz, double f1_hz) {
  const Bins bins = band_bins(g, f0_hz, f1_hz);
  if (bins.first == bins.end) return 0;
  return duty_of(frame_power(g, {bins})[0]);
}

double tdd_frame(const StepGrid &g, double f0_hz, double f1_hz) {
  const Bins bins = band_bins(g, f0_hz, f1_hz);
  if (bins.first == bins.end) return 0;
  return frame_of(frame_power(g, {bins})[0], g.fs());
}

DigitalKind judge(const WideRegion &b) {
  const double w = b.f1_hz - b.f0_hz;
  if (w < kMinWidth || w > kMaxWidth || b.snr_db < kMinSnrDb) return DigitalKind::None;
  // With DJI's frame the gaps are the frame's (duty 0.15-0.9), so being there all the time is not asked for, and the
  // frame already rules out analog FM and Wi-Fi, so less flatness will do (a real DJI at 1-2 m: at 30-40 dB its sides
  // take ~2 MHz each and multipath bows the block by ~11 dB; flat 0.34-0.99). Only DJI's frame, though: analog FM can
  // give a "period" too (from the picture; a real transmitter on R4 gave 1.0-1.3 ms with flat 0.14-0.38), so any
  // other frame has to be as flat as ever.
  if (is_dji_frame(b.frame_s)) return b.flat >= kMinFlatTdd ? DigitalKind::DJI : DigitalKind::None;
  if (b.flat < kMinFlat) return DigitalKind::None;
  if (b.frame_s > 0) return DigitalKind::Other;  // a frame of its own: HDZero has no gaps
  if (b.duty < kMinDuty) return DigitalKind::None;
  if (!on_hdzero_channel((b.f0_hz + b.f1_hz) / 2)) return DigitalKind::Other;
  return w >= kHdzWideMin ? DigitalKind::HDZero : w >= kHdzNarrowMin ? DigitalKind::HDZeroNarrow : DigitalKind::Other;
}

const char *digital_code(DigitalKind k) {
  switch (k) {
  case DigitalKind::HDZero: return "hdzero";
  case DigitalKind::HDZeroNarrow: return "hdzero_narrow";
  case DigitalKind::DJI: return "dji";
  case DigitalKind::Other: return "digital";
  case DigitalKind::None: break;
  }
  return "";
}

const char *digital_label(DigitalKind k) {
  switch (k) {
  case DigitalKind::HDZero: return "HDZero";
  case DigitalKind::HDZeroNarrow: return "HDZero narrow";
  case DigitalKind::DJI: return "DJI";
  case DigitalKind::Other: return "Digital";
  case DigitalKind::None: break;
  }
  return "";
}

std::vector<DigitalSignal> DigitalScan::step(const std::vector<float> &sm, const StepGrid &grid, double centre_hz,
                                             double edge_hz) {
  std::vector<WideRegion> regions = find_wide_regions(sm, grid.fs(), edge_hz);
  std::vector<Bins> bands;
  std::vector<size_t> timed;  // the regions bands[k] is of
  for (size_t i = 0; i < regions.size(); i++) {
    if (!may_be_digital(regions[i])) continue;
    const Bins bins = band_bins(grid, regions[i].f0_hz, regions[i].f1_hz);
    if (bins.first == bins.end) continue;
    bands.push_back(bins);
    timed.push_back(i);
  }
  if (!bands.empty()) {
    const auto power = frame_power(grid, bands);
    for (size_t k = 0; k < timed.size(); k++) {
      regions[timed[k]].duty = duty_of(power[k]);
      regions[timed[k]].frame_s = frame_of(power[k], grid.fs());
    }
  }

  std::vector<WideRegion> whole;
  for (auto &r : regions) {
    r.f0_hz += centre_hz;  // on the band from here on
    r.f1_hz += centre_hz;
    KP_LOGD("digital: %.1f MHz step, block %.1f..%.1f MHz%s%s, %.1f dB, flat %.2f, duty %.2f, frame %.3f ms",
            centre_hz / 1e6, r.f0_hz / 1e6, r.f1_hz / 1e6, r.clip_lo ? " [cut low]" : "", r.clip_hi ? " [cut high]" : "",
            r.snr_db, r.flat, r.duty, r.frame_s * 1e3);
    if (!r.clip_lo && !r.clip_hi) whole.push_back(r);
  }
  // Cut at this step's lower edge and at the upper edge of the step just below it (Sweep::plan goes upwards), and
  // overlapping: one signal. The first step of a pass is below the step before it, the last of the pass before.
  if (!_prev.empty() && centre_hz > _prev_centre && centre_hz - _prev_centre <= 2 * edge_hz)
    for (const auto &hi : regions)
      for (const auto &lo : _prev) {
        if (!lo.clip_hi || lo.clip_lo || !hi.clip_lo || hi.clip_hi) continue;
        if (std::min(lo.f1_hz, hi.f1_hz) - std::max(lo.f0_hz, hi.f0_hz) < 1e6) continue;
        // Flat and continuous: the better half's. Each half sees the signal beside its filter's edge and one of the
        // two steps may have it over its DC point: with a real HDZero transmitter on R1, upstream's step at 5650 MHz
        // measured flat 0.6-0.8 and duty 0.5-0.9 and the one at 5670 MHz 0.96 and 1.00, scan after scan. Wi-Fi still
        // stays out: it comes in bursts in both halves.
        WideRegion j = lo;
        j.f1_hz = hi.f1_hz;
        j.clip_hi = false;
        j.snr_db = std::max(lo.snr_db, hi.snr_db);
        j.flat = std::max(lo.flat, hi.flat);
        j.duty = std::max(lo.duty, hi.duty);
        j.frame_s = lo.frame_s > 0 ? lo.frame_s : hi.frame_s;
        whole.push_back(j);
      }
  _prev = std::move(regions);
  _prev_centre = centre_hz;

  std::vector<DigitalSignal> out;
  for (const auto &b : whole) {
    DigitalSignal s;
    s.kind = judge(b);
    if (s.kind == DigitalKind::None) continue;
    s.centre_hz = (b.f0_hz + b.f1_hz) / 2;
    s.width_hz = b.f1_hz - b.f0_hz;
    s.snr_db = b.snr_db;
    KP_LOGD("digital: %s at %.1f MHz, %.1f MHz wide, %.1f dB", digital_code(s.kind), s.centre_hz / 1e6,
            s.width_hz / 1e6, s.snr_db);
    out.push_back(s);
  }
  return out;
}

void DigitalMap::add(const std::vector<DigitalSignal> &found, int64_t now_ms) {
  for (DigitalSignal s : found) {
    bool known = false;
    for (auto it = _kept.begin(); it != _kept.end();) {
      if (std::fabs(it->centre_hz - s.centre_hz) < kSameHz) {
        known = known || it->kind == s.kind;
        it = _kept.erase(it);
      } else {
        ++it;
      }
    }
    // once per signal, not at every step that finds it again
    if (!known)
      KP_LOGI("digital: %s at %.1f MHz, %.1f MHz wide, %.1f dB", digital_code(s.kind), s.centre_hz / 1e6,
              s.width_hz / 1e6, s.snr_db);
    s.seen_ms = now_ms;
    _kept.push_back(s);
  }
}

void DigitalMap::forget(double lo_hz, double hi_hz, int64_t older_than_ms) {
  _kept.erase(std::remove_if(_kept.begin(), _kept.end(),
                             [&](const DigitalSignal &s) {
                               return s.centre_hz >= lo_hz && s.centre_hz <= hi_hz && s.seen_ms < older_than_ms;
                             }),
              _kept.end());
}

const DigitalSignal *digital_for(const std::vector<DigitalSignal> &verdicts, const Carrier &c) {
  const DigitalSignal *best = nullptr;
  double best_off = DigitalMap::kSameHz;
  for (const auto &v : verdicts) {
    const double off = std::fabs(v.centre_hz - c.mhz * 1e6);
    // About as wide as the signal judged: not a carrier the band picture has run together with its neighbour, nor a
    // narrow one that only shares its centre. (A transmitter of another kind that takes its place on the same
    // channel is not told apart by this: it waits for the verdict to go.)
    const double ratio = c.width_mhz * 1e6 / v.width_hz;
    if (off < best_off && ratio > 0.5 && ratio < 2) {
      best = &v;
      best_off = off;
    }
  }
  return best;
}

}  // namespace kp
