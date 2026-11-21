// kepyur decoder: one line of video out of the sample rings into a row of pixels.
#pragma once
#include <complex>
#include <cstdint>
#include <vector>

#include "rings.h"
#include "video.h"

namespace kp {

// A line of video between the decoder's sync tracking and the picture: where it is and what was measured on it.
struct LineInfo {
  double ls = -1;      // its start (the hsync's leading edge), in samples
  int64_t k = -1;      // its row in the field's picture; outside 0..active-1 it is not picture
  int field = -1;
  bool col = false;    // drawn in colour
  bool pal_v = false;  // PAL: this line's V is switched
  bool ok = false;     // usable as a comb neighbour: locked, in colour, burst on the PLL's predicted phase
  bool hit = false;    // sparkle repair: still badly hit after patching (SparkleRepair::mend_rows)
  bool fade = false;   // adaptive comb: SampleRings::fade holds this line's fade
  uint64_t epoch = 0;  // which stretch of unbroken stream it is in (lines across a jump don't comb)
  double phase = 0;    // the burst's reference phase and amplitude after this line's burst
  float mag = 0;
};

// What a line is drawn with besides itself: the decoder's levels and timing as of now.
struct DrawParams {
  float sign, tip, blank;  // polarity; sync tip and blanking levels, MHz
  double T;                // line period, samples
  double cdel;             // chroma filter delay minus luma filter delay, samples
};

class LineDrawer {
public:
  LineDrawer(int width, double fs);
  // comb_w: the colour oscillator's frequency, rad per sample. Forgets the line before (PAL-D).
  void set_std(const VideoStd &s, double comb_w);
  // The line into row (`width` RGBX pixels): luma between the two samples either side of each pixel, chroma
  // likewise, turned onto the burst's phase and scaled, PAL-D averaged with the line before, to RGB. Pixels
  // whose samples are not in the rings (past the stream's end, or too far back) are black.
  // up / dn: comb this line's chroma with the line above / below (NTSC, DecoderOpts::comb). The subcarrier is
  // 227.5 x the line rate, so at the same place one line away chroma comes back with its sign flipped while
  // luma detail does not: combing cancels the luma near the subcarrier that the chroma filter lets through as
  // rainbows. Per pixel a neighbour only counts as far as its luma there matches this line's, so the top and
  // bottom edges of OSD glyphs fall back to the plain filter instead of keeping half the rainbow.
  void draw(uint32_t *row, const SampleRings &R, const LineInfo &l, bool up, bool dn, const DrawParams &p);

private:
  void comb_weights(const SampleRings &R, double ls, float yscale, bool up, bool dn, bool fade, const DrawParams &p);
  // pixels [x0, x1) of the row, from the samples at base (the picture's start) on
  void draw_grey(uint32_t *row, const SampleRings &R, double base, float yscale, int x0, int x1, const DrawParams &p);
  void draw_colour(uint32_t *row, const SampleRings &R, double base, float yscale, std::complex<float> rot, float k_uv,
                   bool comb_on, bool pal_v, int x0, int x1, const DrawParams &p);

  static constexpr float kCombFull = 0.08f, kCombNone = 0.25f;  // luma mismatch, of black-to-white
  static constexpr int kPad = 3;  // entries past the row's end in what the draw loops read per pixel (set_std)
  int _w;
  double _fs;
  double _a0 = 0, _aw = 0;  // the picture's start after the line's, and its width, in samples
  bool _pal = false;
  double _comb_w = 0;
  int _fade_back, _fade_step;  // where comb_weights reads the adaptive comb's fade, in samples before the chroma
  std::vector<double> _px_off;  // pixel x is _px_off[x] samples into the line's picture
  std::vector<float> _prev_u, _prev_v;  // PAL-D: the previous line's U/V
  std::vector<float> _comb_m;  // the comb on every other pixel: chroma becomes c * (1 - _comb_m) - _comb_o
  std::vector<std::complex<float>> _comb_o;
  std::vector<float> _px_cm, _px_cr, _px_ci;  // ... and for each pixel (chroma * _px_cm - _px_c{r,i})
};

}  // namespace kp
