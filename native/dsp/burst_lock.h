// fpvDROID decoder: the colour reference. A per-line PLL on the burst tracks the transmitter's subcarrier phase
// (and its small frequency error) against the picture filters' free-running oscillator, and a colour killer says
// whether there is colour to decode. PAL: the burst's +-45 degree swing gives each line's V switch.
//
// fpvDROID: colour from a noisy signal (Way::mean). One line's burst is then mostly noise (an FM signal's noise
// grows with the frequency, and the burst sits at the top of the band): a reference that follows each burst a
// fifth of the way swings with the noise, and the picture gets bands of green and purple. But the subcarrier is as
// steady as the line period, and the hold has every line's start whether its hsync was found or not (line_hold.h):
// so the bursts of the last kLines lines, each turned back by the reference predicted for its line, are averaged,
// and the reference follows that mean slowly. How much of the bursts' size is left in the mean (1 without noise,
// about 0.1 with nothing but noise) says whether there is colour to be had: under kOff there is none. Above it the
// picture's colour is drawn smoothed along the line and down the field (calm: LineInfo::calm), and its saturation
// is turned down by the share of signal in what the colour path then gives: the burst's size, squared, against
// that and what is left of the path's noise (kNoise of a sample's). The picture's colours stay where they are,
// paler, and the coloured snow is as much paler.
// NTSC only so far: PAL's swinging burst wants the lines' V switch from more than one line as well. PAL keeps the
// cut (Way::cut): no colour while the sync is noisy.
#pragma once
#include <complex>
#include <cstdint>

#include "rings.h"
#include "sync_noise.h"

namespace kp {

// How calm a noisy signal's colour is drawn (LineDrawer::draw_colour_calm): U and V smoothed down the field and
// along the line.
struct Calm {
  float share = 0;  // down the field: the line's own share of what is drawn, the rest being what the lines above
                    // it had (0: not calm, the colour as it comes)
  int cells = 0;    // along the line: the mean over so many cells either side (a cell: a few pixels)
};

class BurstLock {
public:
  // can_average: the decoder has every line's start, found or not (DecoderOpts::hold), so that a noisy signal's
  // bursts can be averaged
  BurstLock(double fs, bool can_average) : _fs(fs), _can_average(can_average) {}
  // A new standard: no colour until its bursts say so, and no reference yet.
  void set_std(bool pal) {
    _pal = pal;
    _on = false;
    _good = 0;
    _last_line = -1;
    _snr = 0;
  }
  // How noisy the sync is now (SyncNoise::snr, as often as it has a new one). The burst of a signal whose sync is
  // noisier than kSnrOff is not one to take colour from line by line: its phase is mostly the noise's. Judged over
  // a tenth of a second or so, as the front end's channel filter is.
  void sync_snr(float snr) {
    _noise.update(snr, 0.02f);
    _snr = _snr > 0 ? _snr + 0.1f * (snr - _snr) : snr;
  }
  // The sync is that noisy, and the bursts are averaged: then a line whose hsync was not found has colour too, if
  // the decoder has its start.
  bool averaging() const { return way() == Way::mean; }
  // The burst of line `line`, which began at ls: chroma over the burst window (NTSC 5.3-7.8 us, PAL 5.6-7.85 us
  // after the hsync edge; the chroma filter delays it ~0.35 us). The reference follows it. depth: the sync depth,
  // which a burst's size is judged by. pal_v: the line's V switch (PAL). Is the line in colour? (False too when
  // its burst is not in the rings.)
  bool line(const SampleRings &R, double ls, int64_t line, float depth, bool &pal_v);

  bool on() const { return _on; }                    // colour is being decoded
  double phase() const { return _phase; }            // the reference phase after the last burst (rad) ...
  float size() const { return _size; }               // ... and the burst amplitude the colour is drawn against
  Calm calm() const;                                 // ... and how calm it is drawn (LineInfo::calm)
  float err() const { return _err; }                 // the bursts' mean |phase error| against the reference (rad)
  float last_err() const { return _e; }              // ... and the last burst's

private:
  // What a line's burst is taken for.
  //  each: the reference follows every burst (the sync is clean);
  //  cut:  the same, and there is no colour to be had (the sync is noisy);
  //  mean: the sync is noisy, and the reference follows the mean of many lines' bursts (see above).
  enum class Way { each, cut, mean };
  Way way() const { return !_noise.noisy ? Way::each : _can_average && !_pal ? Way::mean : Way::cut; }

  static constexpr float kSnrOff = 12, kSnrOn = 16;
  static constexpr int kLines = 128, kMin = 32;
  static constexpr float kOn = 0.4f, kOff = 0.25f;
  static constexpr double kStep = 0.05, kDrift = 0.0005;
  static constexpr float kNoise = 0.3f;
  static constexpr Calm kMeanCalm{0.1f, 3};  // Way::mean's colour is drawn this calm
  // A moderately noisy signal's colour, line by line still (Way::each), is drawn calm too (NTSC), the more the
  // noisier the sync: not at all at a sync snr of kEachHi and over, at the most (kEachCalm) at kEachLo and under.
  // Measured on a still scene against the mean of 200 of its pictures (its true colours): one recording at sync
  // snr 27 had its colour error go from 13.1 to 5.2 at that strength (stronger smoothing gained little more and
  // blurred more), a clean one with noise added from 13.7 to 5.3 at snr 29 and from 8.2 to 3.8 at 44, while the
  // blur at colour edges doubles (2.9 to 5.7) at any snr. A clean signal (snr 57 there) keeps its sharp colour.
  static constexpr Calm kEachCalm{0.3f, 2};
  static constexpr float kEachHi = 50, kEachLo = 30;

  bool follow_mean(std::complex<float> b, float mag, float nv, double pred, int64_t line);
  void note(int64_t line, float mag, float e);
  bool killer(bool ok);

  double _fs;
  bool _can_average;
  bool _pal = false;
  NoisyLatch _noise{kSnrOff, kSnrOn};  // the sync is too noisy for a line's burst
  double _phase = 0, _dphi = 0;  // the reference phase (rad) and its drift per line
  int64_t _last_line = -1;       // the line of the last burst (-1: none yet)
  float _mag = 0, _err = 1;      // the bursts' amplitude (MHz) and mean |phase error| (rad), smoothed
  float _e = 0;
  float _size = 0;
  float _snr = 0;                // the sync's snr lately (0: not known)
  bool _on = false;
  int _good = 0;                 // the colour killer's count: up with a good burst, down two with a bad one
  // Way::mean
  bool _mean = false;            // the last burst was taken that way
  std::complex<float> _z{};      // the mean of the last kLines bursts, against the reference ...
  float _zmag = 0;               // ... of their sizes ...
  float _nvar = 0;               // ... and of the colour path's noise, a sample: mean square
  int _zn = 0;                   // how many are in it
};

}  // namespace kp
