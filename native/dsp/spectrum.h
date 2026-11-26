// fpvDROID: power spectrum and analog-video signal finding (the scanner's DSP half).
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "video.h"

namespace kp {

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
  std::vector<float> _win;
  std::vector<cf> _buf, _tmp;
  std::vector<double> _acc;
  size_t _fill = 0;
  int _frames = 0;
};

void fft_inplace(cf *x, int n);  // radix-2, n a power of two, forward

// A hump in the spectrum that looks like an FM video carrier (3.5..24 MHz wide at floor + 6 dB).
struct Hump {
  double offset_hz;  // centroid, relative to the stream centre
  double width_hz;
  float peak_db, snr_db;
};
// Humps fully inside +-edge_hz of the centre.
std::vector<Hump> find_humps(const std::vector<float> &db, double fs, double edge_hz);

// A signal that runs past one edge of +-edge_hz, which find_humps leaves out: with scan steps 20 MHz apart a signal
// is sure to be whole in one only up to 13 MHz wide, and a VTX close by is 15-20. The same runs over the same floor
// as find_humps, at least 3.5 MHz of each inside the span; f0..f1 that part (relative to the centre), snr_db its
// median over the floor. One past both edges, wider than the span, is not a VTX and is left out; with the scan's
// span (+-16.5 MHz at 40 MS/s) there is none anyway: it would hold over three quarters of the bins, and the floor is
// their lower quartile. A signal that wide (a 40 or 80 MHz Wi-Fi channel) leaves nothing over the floor.
struct CutSignal {
  double f0_hz = 0, f1_hz = 0;
  bool cut_hi = false;  // it goes on past the top edge; else past the bottom one
  float snr_db = 0;
};
std::vector<CutSignal> find_cut_signals(const std::vector<float> &db, double fs, double edge_hz);

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
  // One retune's spectrum (Psd::db(): fftshifted, fs wide, centred on the hardware centre centre_hz). Its bins
  // within +-use_hz of the centre go in, all but the DC point's (the chip's DC filter digs a notch there). A bin
  // already seen in the last few seconds is averaged with the new value, so a burst of Wi-Fi does not flicker.
  void add(const std::vector<float> &db, double fs, double centre_hz, double use_hz, int64_t now_ms);
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
