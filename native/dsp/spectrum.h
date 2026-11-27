// fpvDROID: power spectrum and analog-video signal finding (the scanner's DSP half).
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "video.h"

namespace kp {

// A forward radix-2 FFT of one size (a power of two), its bit-reversed order and its twiddles worked out once: for
// the many FFTs of one size a sweep step takes (Psd's ~940 of 1024 points, StepGrid's 1875 of 256).
class Fft {
public:
  explicit Fft(int n);
  int size() const { return _n; }
  void run(cf *x) const;  // in place
  // out = the FFT of in[i] * win[i] (n of each), the window put on as the samples are put in bit-reversed order
  void run(const cf *in, const float *win, cf *out) const;

private:
  void butterflies(cf *x) const;
  int _n;
  std::vector<int> _rev;
  std::vector<cf> _tw;  // stage by stage: a stage of span 2h has its h twiddles exp(-2 pi i k / 2h) from _tw[h - 1]
};

// An FFT now and then (the tools' transmitters): Fft(n).run(x), its plan made for the one call.
void fft_inplace(cf *x, int n);

// The Hann window of n points (periodic: 0.5 - 0.5 cos(2 pi i / n)), as every spectrum here takes it.
std::vector<float> hann(int n);

// Averaged power spectrum, fftshifted (bin 0 = -fs/2), in dB. Hann window, 50 % overlap.
class Psd {
public:
  explicit Psd(int n = 1024);
  void reset();
  void add(const cf *x, size_t count);  // any count; leftovers carry to the next call
  int frames() const { return _frames; }
  int size() const { return _n; }
  std::vector<float> db() const;

private:
  int _n;
  Fft _fft;
  std::vector<float> _win;
  std::vector<cf> _buf, _tmp;
  std::vector<double> _acc;
  size_t _fill = 0;
  int _frames = 0;
};

// The chip's DC filter digs a notch this wide either side of a retune's centre: left out of what is measured there.
constexpr double kDcNotchHz = 0.75e6;

// A spectrum (Psd::db(), bins of bin_hz) median-smoothed over +-0.3 MHz: the DC line and narrow spurs taken out. What
// every measure of a retune's spectrum reads: a sweep step smooths its spectrum once for all of them.
std::vector<float> smooth_db(const std::vector<float> &db, double bin_hz);

// The median of v (the upper one of an even count); 0 for none.
float median_of(std::vector<float> v);

// A hump in the spectrum that looks like an FM video carrier: kHumpMinHz..kHumpMaxHz wide at floor + 6 dB (a VTX
// next to the phone is about 16 MHz wide).
inline constexpr double kHumpMinHz = 3.5e6, kHumpMaxHz = 24e6;
struct Hump {
  double offset_hz;  // centroid, relative to the stream centre
  double width_hz;
  float peak_db, snr_db;
};
// A signal that runs on past one edge of +-edge_hz, so that it is no hump: f0..f1 the part of it inside (relative to
// the centre, at least kHumpMinHz), snr_db that part's median over the floor.
struct CutSignal {
  double f0_hz = 0, f1_hz = 0;
  bool cut_hi = false;  // it goes on past the top edge; else past the bottom one
  float snr_db = 0;
};
// One spectrum's signals over one floor: the humps fully inside +-edge_hz of the centre, and the signals cut by one
// edge. One past both edges is wider than the span, no VTX, and left out. (With edge_hz over 3/8 of fs there is none:
// it would hold over three quarters of the bins, and the floor is their lower quartile.)
struct StepSignals {
  std::vector<Hump> whole;
  std::vector<CutSignal> cut;
};
StepSignals find_step_signals(const std::vector<float> &db, double fs, double edge_hz);
// Its humps alone.
std::vector<Hump> find_humps(const std::vector<float> &db, double fs, double edge_hz);

// Live SNR of the carrier being watched (dB): the strongest point of the smoothed spectrum against the noise
// floor (10th percentile) inside +-span_hz of the carrier, which sits carrier_hz from the stream centre (0 unless
// the hardware centre is put beside it). pass_hz > 0 also keeps the span inside +-pass_hz of the stream centre, the
// band the RX filter passes (an offset carrier's span would reach past it otherwise). Comparable with
// Hump::snr_db for a signal that leaves part of the span free; a very wide, close signal reads lower.
float carrier_snr(const std::vector<float> &db, double fs, double span_hz, double carrier_hz = 0, double pass_hz = 0);

// The bands as one picture, for the spectrum page: the spectrum of every retune (a scan step, a sweep step) laid on
// one grid, each in dB over its own noise floor. Only the floor makes two steps comparable: the chip's AGC picks a
// gain of its own at every retune. Not thread-safe.
class BandMap {
public:
  // What get() gives for a bin nothing has been seen in. (Not NaN: this library is built with -ffast-math, under
  // which a test for NaN may be compiled away.)
  static constexpr float kNone = -1000.f;

  BandMap(double lo_hz, double hi_hz, double bin_hz);
  // One retune's spectrum through smooth_db (Psd::db(): fftshifted, fs wide, centred on the hardware centre
  // centre_hz). Its bins within +-use_hz of the centre go in, all but the DC point's (kDcNotchHz). A bin already seen
  // in the last few seconds is averaged with the new value, so a burst of Wi-Fi does not flicker.
  void add(const std::vector<float> &smoothed, double fs, double centre_hz, double use_hz, int64_t now_ms);
  // Bins in lo..hi last seen before older_than_ms are forgotten.
  void forget(double lo_hz, double hi_hz, int64_t older_than_ms);
  // lo..hi on the grid (first bin at *first_hz): dB over the floor, kNone where nothing has been seen. A gap of up
  // to 3 MHz between seen bins (the DC points of a scan, which steps 20 MHz) is bridged.
  std::vector<float> get(double lo_hz, double hi_hz, double *first_hz) const;
  double bin_hz() const { return _bin; }

private:
  double _lo, _bin;
  std::vector<float> _db;
  std::vector<int64_t> _t;  // when each bin was last written
};

// Something on the air in a BandMap picture: a run at least 2 MHz wide standing 6 dB over the floor (dips under
// that of up to 2 MHz inside it do not split it: weak FM video has them).
struct Carrier {
  double mhz;        // power-weighted centre
  double width_mhz;
  float snr_db;      // its peak over the floor
};
std::vector<Carrier> find_carriers(const std::vector<float> &db, double first_hz, double bin_hz);

// Sync check: is there analog video at this offset? Runs a FrontEnd over the samples.
struct VideoCheck {
  bool video = false;
  const char *std_name = "?";
  float lock = 0;  // locked fraction of lines at the end
  float dev_mhz = 0;  // sync tip to white estimate
};
VideoCheck check_video(const cf *x, size_t n, double fs, double offset_hz);

// Standard analog 5.8 GHz FPV channel names.
struct FpvChannel {
  const char *name;
  double mhz;
};
const std::vector<FpvChannel> &fpv_channels();
// Nearest standard channel within max_off_mhz, or nullptr.
const FpvChannel *nearest_channel(double mhz, double max_off_mhz = 6.0);

}  // namespace kp
