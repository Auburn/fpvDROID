// kepyur decoder: the luma and colour paths. Every sample of discriminator output (MHz) becomes brightness and
// baseband colour in the sample rings, through one of two chains:
//  - the notch chain: luma through a 3 MHz low-pass (two sections) and a notch at the subcarrier (rings ~4 px
//    after every edge);
//  - the luma comb (NTSC, PictureOpts::luma_comb): a gentle 4.8 MHz low-pass, colour cancelled with the line
//    before (no ringing, sharper), and on top of it the adaptive comb (adaptive_comb), which falls back to the notch
//    and fades the colour where the line before is clearly different.
// Colour in both: mixed down by a free-running oscillator at the nominal subcarrier, whose phase belongs to the
// absolute sample index (burst and picture share it), then a 4th-order Butterworth low-pass at 1.2 MHz (U/V
// bandwidth) for each of U and V.
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>

#include "biquad.h"
#include "rings.h"
#include "video.h"

namespace kp {

class PictureFilters {
public:
  PictureFilters(double fs, bool colour);
  void set_std(const VideoStd &s);
  // Before each line's samples: the switches as they are for it, the line period T (samples) and 1 / the sync
  // depth. They hold until the next call.
  // comb3: the 3-line chroma comb is on (PictureOpts::chroma_comb, NTSC). The adaptive comb then only notes its
  // fade (SampleRings::fade), for LineDrawer to apply where that comb finds no matching line.
  void set_line(bool luma_comb, bool adaptive, bool comb3, double T, float inv_depth);
  // The next cnt samples (bad: their click marks, or null) through the chain into the rings; R.n moves on by cnt.
  void run(SampleRings &R, const float *d, const uint8_t *bad, size_t cnt);
  // With the luma comb: the picture [pic0, pic0 + aw) of a line already in the rings worked out again the way
  // the notch chain does it, from what the comb's ring holds of it: brightness through the notch, colour
  // straight from the signal, not faded. whole: all of it; else its left half, blended into the comb's at the
  // middle. For the top line of the picture, which has blanking above it (Decoder::top_line).
  void redo_without_comb(SampleRings &R, double pic0, double aw, bool whole);

  bool luma_comb() const { return _luma_comb; }
  // the adaptive comb is fading colour in the chain ...
  bool colour_fades() const { return _adaptive && !_comb3; }
  // ... or has noted this line's fade for the 3-line comb
  bool fade_noted() const { return _comb3 && _adaptive && _adaptive_lines > 1; }
  double comb_w() const { return _comb_w; }  // the oscillator's frequency, rad per sample
  // chroma filter delay minus luma filter delay, samples. The notch chain was aligned by hand (0.21 us); the
  // comb's luma filter is faster, so colour comes that much later relative to it.
  double cdel() const { return _luma_comb ? _cdel_comb : 0.21e-6 * _fs; }

private:
  void filter_notch(SampleRings &R, const float *d, const uint8_t *bd, size_t cnt);
  void filter_comb(SampleRings &R, const float *d, const uint8_t *bd, size_t cnt);

  double _fs;
  bool _colour;  // DecoderOpts::colour
  bool _luma_comb = false, _adaptive = false, _comb3 = false;  // set_line
  int _adaptive_lines = 0;  // lines since the adaptive comb came on: its filters and ring need one to fill
  Biquad _lp_luma, _lp_luma2, _luma_notch;                // the notch chain
  Biquad _lp_wide, _bp_chroma, _lp_vert, _luma_notch_w;   // the luma comb, and the adaptive comb's two
  Biquad _cl_r1, _cl_r2, _cl_i1, _cl_i2;                  // colour
  std::complex<float> _lo{1, 0}, _lo_step{1, 0};
  double _comb_w = 0;
  uint64_t _prev_back = 1;  // the line before: ring sample n - _prev_back, and _prev_fr of the way to the next one
  float _prev_fr = 0, _inv_depth = 1;
  float _vdiff = 0;
  int _chroma_quiet = 0;  // samples in a row with the colour faded out completely
  double _cdel_comb = 0;
};

}  // namespace kp
