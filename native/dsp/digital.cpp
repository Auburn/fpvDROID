// Ported from FPVDesktopWiFiReceiver's src/kp_ext/digital_scan.cpp (github.com/Pwn3dP4nd4/fpvdesktopwifireceiver,
// GPL-2.0): the thresholds and the reasons for them are upstream's. What is fpvDROID's own: band_duty and tdd_frame
// share their FFTs (frame_power, with an FFT of its own: TimeFft), the timing is worked out only for blocks that may
// be digital, and the verdicts are kept from pass to pass (DigitalMap).
#include "digital.h"

#include <algorithm>
#include <cmath>

#include "log.h"

namespace kp {

namespace {
constexpr float kAboveFloorDb = 6;     // the band a block takes: what stands over the floor + 6 dB (as find_humps)
constexpr float kMinSnrDb = 8;         // a block's median level over the floor
constexpr double kMinVisible = 6e6;    // a narrower piece is left out (and not joined with the next step's either)
constexpr double kJoinGap = 0.5e6;     // narrow gaps inside a block (a null subcarrier at DC, the chip's DC notch)
constexpr double kClipMargin = 0.3e6;  // this close to the edge of the span: cut
constexpr float kMinFlat = 0.8f;       // OFDM ~0.95; FM with a picture, less
constexpr float kMinFlatTdd = 0.3f;    // with DJI's frame (see DigitalScan::step)
constexpr float kMinDuty = 0.9f;       // HDZero sends without a break; Wi-Fi comes in bursts
constexpr double kMinWidth = 12e6, kMaxWidth = 31e6;
constexpr double kHdzWideMin = 22e6, kHdzNarrowMin = 14e6;  // 27 and 17 MHz as specified
constexpr double kHdzMaxOffMhz = 2;    // measured centre against the channel (the transmitter's and adapter's crystals)
// TDD frames (tdd_frame)
constexpr double kTddMinFrame = 1e-3, kTddMaxFrame = 4e-3;  // at least 3 frames in a step's 12 ms
constexpr double kTddOnMin = 0.15, kTddOnMax = 0.95;        // neither almost silent nor continuous (HDZero)
constexpr double kTddMinRepeat = 0.7;  // how much the pattern repeats a period later, chance taken out (0..1)
constexpr double kDjiFrame = 2.5e-3, kDjiFrameTol = 0.1e-3;
// the timing measures: band power in kTimeN-point FFT frames (6.4 us at 40 MS/s), summed over kDutyFrames for
// band_duty (~0.1 ms slices) and over kTddFrames for tdd_frame (12.8 us slices)
constexpr int kTimeN = 256, kDutyFrames = 16, kTddFrames = 2;

// Median-smoothed spectrum over +-0.3 MHz, as spectrum.cpp's: takes out the DC line and narrow spurs.
std::vector<float> median_smooth(const std::vector<float> &db, double bin) {
  const int n = static_cast<int>(db.size());
  const int hw = std::max(2, static_cast<int>(0.3e6 / bin));
  std::vector<float> sm(n), win;
  for (int i = 0; i < n; i++) {
    win.clear();
    for (int k = std::max(0, i - hw); k <= std::min(n - 1, i + hw); k++) win.push_back(db[k]);
    std::nth_element(win.begin(), win.begin() + win.size() / 2, win.end());
    sm[i] = win[win.size() / 2];
  }
  return sm;
}

float median_of(std::vector<float> v) {
  if (v.empty()) return 0;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

// The bins of a kTimeN-point FFT (not fftshifted) in the band f0..f1 less 10 % at each side (where the edges fall).
std::vector<int> band_bins(double fs, double f0_hz, double f1_hz) {
  const double w = f1_hz - f0_hz;
  const double a = f0_hz + 0.1 * w, b = f1_hz - 0.1 * w;
  std::vector<int> bins;
  for (int k = 0; k < kTimeN; k++) {
    const double f = (k < kTimeN / 2 ? k : k - kTimeN) * fs / kTimeN;
    if (f >= a && f <= b) bins.push_back(k);
  }
  return bins;
}

// A Hann-windowed kTimeN-point FFT (as fft_inplace gives it) with its window, its bit-reversed order and its twiddles
// worked out once: the timing measures take 1875 of these a step, and fft_inplace, which works its twiddles out as it
// goes, takes about twice as long.
struct TimeFft {
  float win[kTimeN];
  int rev[kTimeN];
  cf tw[kTimeN / 2];

  TimeFft() {
    for (int i = 0; i < kTimeN; i++) {
      win[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * M_PI * i / kTimeN));
      int r = 0;
      for (int b = 1, rb = kTimeN / 2; b < kTimeN; b <<= 1, rb >>= 1) r |= i & b ? rb : 0;
      rev[i] = r;
    }
    for (int k = 0; k < kTimeN / 2; k++)
      tw[k] = cf(static_cast<float>(std::cos(-2 * M_PI * k / kTimeN)), static_cast<float>(std::sin(-2 * M_PI * k / kTimeN)));
  }
  // out = FFT of src[0..kTimeN) windowed
  void run(const cf *src, cf *out) const {
    for (int i = 0; i < kTimeN; i++) out[rev[i]] = src[i] * win[i];
    for (int half = 1, step = kTimeN / 2; half < kTimeN; half <<= 1, step >>= 1)
      for (int i = 0; i < kTimeN; i += 2 * half)
        for (int k = 0; k < half; k++) {
          const cf u = out[i + k], v = out[i + k + half] * tw[k * step];
          out[i + k] = u + v;
          out[i + k + half] = u - v;
        }
  }
};

// The power of each band (its bins) in each kTimeN-sample frame of x, frames one after another from x[0], Hann
// window: out[b][i] is band b in frame i. band_duty and tdd_frame are both sums of these over a few frames (upstream
// worked out each with FFTs of its own, the same ones), and the bands of a step's blocks share each frame's FFT:
// 1875 FFTs for a step's 12 ms, however many blocks it has.
std::vector<std::vector<double>> frame_power(const cf *x, size_t n, const std::vector<std::vector<int>> &bands) {
  static const TimeFft fft;
  const size_t frames = n / kTimeN;
  std::vector<std::vector<double>> out(bands.size(), std::vector<double>(frames, 0.0));
  std::vector<cf> buf(kTimeN);
  for (size_t f = 0; f < frames; f++) {
    fft.run(x + f * kTimeN, buf.data());
    for (size_t b = 0; b < bands.size(); b++) {
      double p = 0;
      for (int k : bands[b]) p += std::norm(buf[k]);
      out[b][f] = p;
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
  const double slice_s = kTimeN * kTddFrames / fs;
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
// digital by itself, so its FFTs are not done; as one half of a signal cut between two steps, the other half's timing
// is the one taken (upstream took the better of the two halves' figures, measured for any block).
bool may_be_digital(const WideRegion &r) {
  const double w = r.f1_hz - r.f0_hz;
  const bool cut = r.clip_lo || r.clip_hi;
  return r.snr_db >= kMinSnrDb && r.flat >= kMinFlatTdd && w <= kMaxWidth && (cut || w >= kMinWidth);
}
}  // namespace

std::vector<WideRegion> find_wide_regions(const std::vector<float> &db, double fs, double edge_hz) {
  std::vector<WideRegion> out;
  const int n = static_cast<int>(db.size());
  if (n < 64) return out;
  const double bin = fs / n;
  const std::vector<float> sm = median_smooth(db, bin);
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

float band_duty(const cf *x, size_t n, double fs, double f0_hz, double f1_hz) {
  const auto bins = band_bins(fs, f0_hz, f1_hz);
  if (bins.empty()) return 0;
  return duty_of(frame_power(x, n, {bins})[0]);
}

double tdd_frame(const cf *x, size_t n, double fs, double f0_hz, double f1_hz) {
  const auto bins = band_bins(fs, f0_hz, f1_hz);
  if (bins.empty()) return 0;
  return frame_of(frame_power(x, n, {bins})[0], fs);
}

const std::vector<double> &hdzero_channels_mhz() {
  static const std::vector<double> ch = {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,   // R1-R8
                                         5705, 5740, 5760, 5800,                           // E1 F1 F2 F4
                                         5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621};  // L1-L8
  return ch;
}

DigitalKind classify_digital(double centre_hz, double width_hz, double frame_s) {
  if (width_hz < kMinWidth || width_hz > kMaxWidth) return DigitalKind::None;
  if (std::fabs(frame_s - kDjiFrame) <= kDjiFrameTol) return DigitalKind::DJI;
  if (frame_s > 0) return DigitalKind::Other;  // HDZero has no gaps
  bool on_channel = false;
  for (double c : hdzero_channels_mhz()) on_channel = on_channel || std::fabs(centre_hz / 1e6 - c) <= kHdzMaxOffMhz;
  if (on_channel && width_hz >= kHdzWideMin) return DigitalKind::HDZero;
  if (on_channel && width_hz >= kHdzNarrowMin) return DigitalKind::HDZeroNarrow;
  return DigitalKind::Other;
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

std::vector<DigitalSignal> DigitalScan::step(const std::vector<float> &db, const cf *x, size_t n, double fs,
                                             double centre_hz, double edge_hz) {
  std::vector<WideRegion> regions = find_wide_regions(db, fs, edge_hz);
  std::vector<std::vector<int>> bands;
  std::vector<size_t> timed;  // the regions bands[k] is of
  for (size_t i = 0; i < regions.size(); i++) {
    if (!may_be_digital(regions[i])) continue;
    auto bins = band_bins(fs, regions[i].f0_hz, regions[i].f1_hz);
    if (bins.empty()) continue;
    bands.push_back(std::move(bins));
    timed.push_back(i);
  }
  if (!bands.empty()) {
    const auto power = frame_power(x, n, bands);
    for (size_t k = 0; k < timed.size(); k++) {
      regions[timed[k]].duty = duty_of(power[k]);
      regions[timed[k]].frame_s = frame_of(power[k], fs);
    }
  }

  std::vector<Abs> now;
  for (const auto &r : regions) {
    now.push_back(Abs{centre_hz + r.f0_hz, centre_hz + r.f1_hz, r.clip_lo, r.clip_hi, r.snr_db, r.flat, r.duty,
                      r.frame_s});
    KP_LOGD("digital: %.1f MHz step, block %.1f..%.1f MHz%s%s, %.1f dB, flat %.2f, duty %.2f, frame %.3f ms",
            centre_hz / 1e6, (centre_hz + r.f0_hz) / 1e6, (centre_hz + r.f1_hz) / 1e6, r.clip_lo ? " [cut low]" : "",
            r.clip_hi ? " [cut high]" : "", r.snr_db, r.flat, r.duty, r.frame_s * 1e3);
  }
  std::vector<Abs> whole;
  for (const auto &a : now)
    if (!a.clip_lo && !a.clip_hi) whole.push_back(a);
  // cut here and in the step before, on opposite sides, and overlapping: one signal
  if (_have_prev && std::fabs(centre_hz - _prev_centre) <= edge_hz + _prev_edge) {
    const bool up = centre_hz > _prev_centre;
    for (const auto &a : now)
      for (const auto &p : _prev) {
        const Abs &lo = up ? p : a, &hi = up ? a : p;
        if (!lo.clip_hi || lo.clip_lo || !hi.clip_lo || hi.clip_hi) continue;
        if (std::min(lo.f1, hi.f1) - std::max(lo.f0, hi.f0) < 1e6) continue;
        // Flat and continuous: the better half's. Each half sees the signal beside its filter's edge and one of the
        // two steps may have it over its DC point: with a real HDZero transmitter on R1, upstream's step at 5650 MHz
        // measured flat 0.6-0.8 and duty 0.5-0.9 and the one at 5670 MHz 0.96 and 1.00, scan after scan. Wi-Fi still
        // stays out: it comes in bursts in both halves.
        whole.push_back(Abs{lo.f0, hi.f1, false, false, std::max(lo.snr, hi.snr), std::max(lo.flat, hi.flat),
                            std::max(lo.duty, hi.duty), lo.frame_s > 0 ? lo.frame_s : hi.frame_s});
      }
  }
  _prev = std::move(now);
  _prev_centre = centre_hz;
  _prev_edge = edge_hz;
  _have_prev = true;

  std::vector<DigitalSignal> out;
  for (const auto &a : whole) {
    // With a regular TDD frame the gaps are the frame's (DJI: duty 0.15-0.9), so being there all the time is not
    // asked for, and the frame already rules out analog FM and Wi-Fi, so less flatness will do (a real DJI at 1-2 m:
    // at 30-40 dB its sides take ~2 MHz each and multipath bows the block by ~11 dB; flat 0.34-0.99). Only DJI's
    // frame, though: analog FM can give a "period" too (from the picture; a real transmitter on R4 gave 1.0-1.3 ms
    // with flat 0.14-0.38), so any other frame has to be as flat as ever.
    const bool tdd = a.frame_s > 0;
    const bool dji = std::fabs(a.frame_s - kDjiFrame) <= kDjiFrameTol;
    if (a.snr < kMinSnrDb || a.flat < (dji ? kMinFlatTdd : kMinFlat) || (!tdd && a.duty < kMinDuty)) continue;
    DigitalSignal s;
    s.centre_hz = (a.f0 + a.f1) / 2;
    s.width_hz = a.f1 - a.f0;
    s.snr_db = a.snr;
    s.kind = classify_digital(s.centre_hz, s.width_hz, a.frame_s);
    if (s.kind == DigitalKind::None) continue;
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
