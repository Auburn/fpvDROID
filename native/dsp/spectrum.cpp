#include "spectrum.h"

#include <algorithm>
#include <cmath>

namespace kp {

void fft_inplace(cf *x, int n) {
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(x[i], x[j]);
  }
  for (int len = 2; len <= n; len <<= 1) {
    const double a = -2 * M_PI / len;
    const cf wl(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
    for (int i = 0; i < n; i += len) {
      cf w(1, 0);
      for (int k = 0; k < len / 2; k++) {
        const cf u = x[i + k], v = x[i + k + len / 2] * w;
        x[i + k] = u + v;
        x[i + k + len / 2] = u - v;
        w *= wl;
      }
    }
  }
}

Psd::Psd(int n) : _n(n), _win(n), _buf(n), _tmp(n), _acc(n, 0.0) {
  for (int i = 0; i < n; i++) _win[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * M_PI * i / n));
}

void Psd::reset() {
  std::fill(_acc.begin(), _acc.end(), 0.0);
  _fill = 0;
  _frames = 0;
}

void Psd::add(const cf *x, size_t count) {
  for (size_t i = 0; i < count; i++) {
    _buf[_fill++] = x[i];
    if (_fill < static_cast<size_t>(_n)) continue;
    for (int k = 0; k < _n; k++) _tmp[k] = _buf[k] * _win[k];
    fft_inplace(_tmp.data(), _n);
    for (int k = 0; k < _n; k++) _acc[k] += std::norm(_tmp[k]);
    _frames++;
    // 50 % overlap
    std::copy(_buf.begin() + _n / 2, _buf.end(), _buf.begin());
    _fill = _n / 2;
  }
}

std::vector<float> Psd::db() const {
  std::vector<float> out(_n);
  const double norm = _frames ? 1.0 / _frames : 1.0;
  for (int k = 0; k < _n; k++) {
    const int src = (k + _n / 2) % _n;  // fftshift
    out[k] = static_cast<float>(10 * std::log10(_acc[src] * norm + 1e-20));
  }
  return out;
}

namespace {
// Median-smoothed spectrum: kills the DC line and narrow carriers (~0.6 MHz window).
std::vector<float> smooth_db(const std::vector<float> &db, double bin) {
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
}  // namespace

StepSignals find_step_signals(const std::vector<float> &db, double fs, double edge_hz) {
  const int n = static_cast<int>(db.size());
  const double bin = fs / n;
  // median-smooth, then floor = lower quartile of the smoothed spectrum
  const std::vector<float> sm = smooth_db(db, bin);
  std::vector<float> sorted = sm;
  std::nth_element(sorted.begin(), sorted.begin() + n / 4, sorted.end());
  const float floor = sorted[n / 4];  // lower quartile: robust even when a wide signal fills much of the span
  const float thr = floor + 6.f;
  StepSignals out;
  int i = 0;
  while (i < n) {
    if (sm[i] <= thr) {
      i++;
      continue;
    }
    int j = i;
    while (j < n && sm[j] > thr) j++;
    const double f0 = (i - n / 2) * bin, f1 = (j - 1 - n / 2) * bin;
    const double width = f1 - f0 + bin;
    const bool lo = f0 <= -edge_hz, hi = f1 >= edge_hz;  // it runs past the bottom, the top edge
    if (!lo && !hi && width >= kHumpMinHz && width <= kHumpMaxHz) {
      double sw = 0, sf = 0;
      float pk = -1e9f;
      for (int k = i; k < j; k++) {
        const double p = std::pow(10.0, (sm[k] - floor) / 10.0);
        sw += p;
        sf += p * (k - n / 2) * bin;
        pk = std::max(pk, sm[k]);
      }
      out.whole.push_back(Hump{sf / sw, width, pk, pk - floor});
    } else if (lo != hi) {
      CutSignal c;
      c.f0_hz = std::max(f0, -edge_hz);
      c.f1_hz = std::min(f1 + bin, edge_hz);
      c.cut_hi = hi;
      std::vector<float> in;  // its bins inside the span, for their median
      for (int k = i; k < j; k++) {
        const double f = (k - n / 2) * bin;
        if (f >= c.f0_hz && f < c.f1_hz) in.push_back(sm[k]);
      }
      if (c.f1_hz - c.f0_hz >= kHumpMinHz && !in.empty()) {
        std::nth_element(in.begin(), in.begin() + in.size() / 2, in.end());
        c.snr_db = in[in.size() / 2] - floor;
        out.cut.push_back(c);
      }
    }
    i = j;
  }
  return out;
}

std::vector<Hump> find_humps(const std::vector<float> &db, double fs, double edge_hz) {
  return find_step_signals(db, fs, edge_hz).whole;
}

float carrier_snr(const std::vector<float> &db, double fs, double span_hz, double carrier_hz, double pass_hz) {
  const int n = static_cast<int>(db.size());
  const double bin = fs / n;
  const std::vector<float> sm = smooth_db(db, bin);
  std::vector<float> in;
  float pk = -1e9f;
  for (int i = 0; i < n; i++) {
    const double f = (i - n / 2) * bin;
    if (std::fabs(f - carrier_hz) > span_hz || (pass_hz > 0 && std::fabs(f) > pass_hz)) continue;
    in.push_back(sm[i]);
    pk = std::max(pk, sm[i]);
  }
  if (in.empty()) return 0;
  std::nth_element(in.begin(), in.begin() + in.size() / 10, in.end());
  return pk - in[in.size() / 10];
}

namespace {
constexpr double kDcHz = 0.75e6;      // the DC notch, left out of a retune's spectrum
constexpr int64_t kAverageMs = 5000;  // a bin seen this recently is averaged with the new value
constexpr double kBridgeHz = 3e6;     // the widest gap get() fills in
constexpr size_t kAnchorBins = 4;     // overlap with fresh bins needed to carry their floor over (2 MHz)
constexpr float kAnchorDb = 3.f;      // ... and how far under this step's own quartile it has to be
}  // namespace

BandMap::BandMap(double lo_hz, double hi_hz, double bin_hz)
    : _lo(lo_hz), _bin(bin_hz),
      _db(static_cast<size_t>(std::max(1L, std::lround((hi_hz - lo_hz) / bin_hz) + 1)), kNone), _t(_db.size(), 0) {}

void BandMap::add(const std::vector<float> &db, double fs, double centre_hz, double use_hz, int64_t now_ms) {
  const int n = static_cast<int>(db.size());
  if (n < 16 || fs <= 0) return;
  const double bin = fs / n;
  const std::vector<float> sm = smooth_db(db, bin);
  // the floor: the lower quartile of the part that is used (the RX filter's roll-off beyond it would pull it down)
  std::vector<float> in;
  for (int i = 0; i < n; i++)
    if (std::fabs((i - n / 2) * bin) <= use_hz) in.push_back(sm[i]);
  if (in.size() < 8) return;
  std::nth_element(in.begin(), in.begin() + in.size() / 4, in.end());
  float floor = in[in.size() / 4];
  const int m = static_cast<int>(_db.size());
  const int j0 = std::max(0, static_cast<int>(std::ceil((centre_hz - use_hz - _lo) / _bin)));
  const int j1 = std::min(m - 1, static_cast<int>(std::floor((centre_hz + use_hz - _lo) / _bin)));
  if (j1 < j0) return;
  const int64_t t = std::max<int64_t>(now_ms, 1);  // 0 marks a bin never seen
  // each bin's level in this step's own terms: the mean of the input bins inside it (or the nearest, should none be)
  std::vector<float> level(static_cast<size_t>(j1 - j0 + 1), kNone);
  for (int j = j0; j <= j1; j++) {
    const double off = _lo + j * _bin - centre_hz;
    if (std::fabs(off) < kDcHz) continue;
    int i0 = static_cast<int>(std::ceil((off - _bin / 2) / bin)) + n / 2;
    int i1 = static_cast<int>(std::floor((off + _bin / 2) / bin)) + n / 2;
    if (i1 < i0) i0 = i1 = static_cast<int>(std::lround(off / bin)) + n / 2;
    i0 = std::max(i0, 0);
    i1 = std::min(i1, n - 1);
    if (i1 < i0) continue;
    double s = 0;
    for (int i = i0; i <= i1; i++) s += sm[i];
    level[j - j0] = static_cast<float>(s / (i1 - i0 + 1));
  }
  // A signal wider than most of the span (DJI at 40 MHz, O4 at 60) leaves too little noise for the quartile, which
  // then sits on the signal and takes its middle out of the picture. Where this step overlaps bins that steps of
  // the last few seconds put in, those say where its floor is: the median of its level less theirs (the same
  // signal at another gain). Taken only when well under the quartile: a quartile of noise agrees with it to a dB
  // or so, and taking the lower of the two every time would walk the floor down, step after step.
  std::vector<float> d;
  for (int j = j0; j <= j1; j++)
    if (level[j - j0] != kNone && _t[j] > 0 && t - _t[j] < kAverageMs) d.push_back(level[j - j0] - _db[j]);
  if (d.size() >= kAnchorBins) {
    std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
    if (d[d.size() / 2] < floor - kAnchorDb) floor = d[d.size() / 2];
  }
  for (int j = j0; j <= j1; j++) {
    if (level[j - j0] == kNone) continue;
    const float v = level[j - j0] - floor;
    _db[j] = _t[j] > 0 && t - _t[j] < kAverageMs ? 0.5f * (_db[j] + v) : v;
    _t[j] = t;
  }
}

void BandMap::forget(double lo_hz, double hi_hz, int64_t older_than_ms) {
  const int m = static_cast<int>(_db.size());
  const int j0 = std::max(0, static_cast<int>(std::floor((lo_hz - _lo) / _bin)));
  const int j1 = std::min(m - 1, static_cast<int>(std::ceil((hi_hz - _lo) / _bin)));
  for (int j = j0; j <= j1; j++)
    if (_t[j] > 0 && _t[j] < older_than_ms) {
      _t[j] = 0;
      _db[j] = kNone;
    }
}

std::vector<float> BandMap::get(double lo_hz, double hi_hz, double *first_hz) const {
  const long m = static_cast<long>(_db.size());
  const long j0 = std::clamp(std::lround((lo_hz - _lo) / _bin), 0L, m - 1);
  const long j1 = std::clamp(std::lround((hi_hz - _lo) / _bin), j0, m - 1);
  std::vector<float> out(static_cast<size_t>(j1 - j0 + 1));
  for (long j = j0; j <= j1; j++) out[j - j0] = _t[j] > 0 ? _db[j] : kNone;
  const int maxgap = static_cast<int>(kBridgeHz / _bin);
  int last = -1;
  for (int k = 0; k < static_cast<int>(out.size()); k++) {
    if (out[k] == kNone) continue;
    if (last >= 0 && k - last > 1 && k - last - 1 <= maxgap)
      for (int q = last + 1; q < k; q++) out[q] = out[last] + (out[k] - out[last]) * (q - last) / (k - last);
    last = k;
  }
  if (first_hz) *first_hz = _lo + j0 * _bin;
  return out;
}

std::vector<Carrier> find_carriers(const std::vector<float> &db, double first_hz, double bin_hz) {
  constexpr float kOver = 6.f;           // dB over the floor, as find_humps
  constexpr double kMinWidth = 2e6;
  const int n = static_cast<int>(db.size());
  const int maxdip = std::max(1, static_cast<int>(2e6 / bin_hz));
  std::vector<Carrier> out;
  int i = 0;
  while (i < n) {
    if (db[i] <= kOver) {  // kNone too
      i++;
      continue;
    }
    int end = i + 1;  // one past the last bin over the threshold, dips of up to maxdip bins taken in
    for (int k = end; k < n && k - end < maxdip; k++)
      if (db[k] > kOver) end = k + 1;
    if ((end - i) * bin_hz >= kMinWidth) {
      double sw = 0, sk = 0;
      float pk = kOver;
      for (int k = i; k < end; k++) {
        if (db[k] <= kOver) continue;  // a dip does not pull the centre
        const double p = std::pow(10.0, db[k] / 10.0);
        sw += p;
        sk += p * k;
        pk = std::max(pk, db[k]);
      }
      out.push_back(Carrier{(first_hz + sk / sw * bin_hz) / 1e6, (end - i) * bin_hz / 1e6, pk});
    }
    i = end;
  }
  return out;
}

VideoCheck check_video(const cf *x, size_t n, double fs, double offset_hz) {
  DecoderOpts o;
  FrontEnd fe(fs, offset_hz, nullptr, o);
  fe.feed(x, n);
  const DecoderStats s = fe.stats();
  VideoCheck r;
  r.video = s.acquisitions > 0 && s.lines > 200 && s.lock_recent > 0.6f;
  r.std_name = s.std_name;
  r.lock = s.lock_recent;
  r.dev_mhz = 3.5f * (s.blank - s.tip);
  return r;
}

const std::vector<FpvChannel> &fpv_channels() {
  static const std::vector<FpvChannel> ch = {
      {"A1", 5865}, {"A2", 5845}, {"A3", 5825}, {"A4", 5805}, {"A5", 5785}, {"A6", 5765}, {"A7", 5745}, {"A8", 5725},
      {"B1", 5733}, {"B2", 5752}, {"B3", 5771}, {"B4", 5790}, {"B5", 5809}, {"B6", 5828}, {"B7", 5847}, {"B8", 5866},
      {"E1", 5705}, {"E2", 5685}, {"E3", 5665}, {"E4", 5645}, {"E5", 5885}, {"E6", 5905}, {"E7", 5925}, {"E8", 5945},
      {"F1", 5740}, {"F2", 5760}, {"F3", 5780}, {"F4", 5800}, {"F5", 5820}, {"F6", 5840}, {"F7", 5860}, {"F8", 5880},
      {"R1", 5658}, {"R2", 5695}, {"R3", 5732}, {"R4", 5769}, {"R5", 5806}, {"R6", 5843}, {"R7", 5880}, {"R8", 5917},
      {"L1", 5362}, {"L2", 5399}, {"L3", 5436}, {"L4", 5473}, {"L5", 5510}, {"L6", 5547}, {"L7", 5584}, {"L8", 5621},
  };
  return ch;
}

const FpvChannel *nearest_channel(double mhz, double max_off_mhz) {
  const FpvChannel *best = nullptr;
  double bd = max_off_mhz;
  // Raceband first on ties: it is by far the most common
  static const char order[] = {'R', 'F', 'A', 'B', 'E', 'L'};
  for (char band : order)
    for (const auto &c : fpv_channels()) {
      if (c.name[0] != band) continue;
      const double d = std::fabs(c.mhz - mhz);
      if (d < bd - 0.01) {
        bd = d;
        best = &c;
      }
    }
  return best;
}

}  // namespace kp
