// kepyur decoder: sparkle repair (set_repair). The white / black sparkles of a weak signal are FM "clicks": the
// carrier's envelope collapses for a moment and its phase slips a turn. The front end marks those instants
// (ClickDetector); the decoder, which draws each line about a line after it arrives, patches the picture over
// each mark and redraws a badly hit row from the rows above and below it (SparkleRepair). From Play RX.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "rings.h"

namespace kp {

// The front end's half, on demod()'s thread. For each call's samples: begin(), envelope() as they are made,
// then mark().
//
// A click is where the envelope drops under -9 dB of its recent level (~2 ms) for a moment AND the phase slips
// about one whole turn across that dip (the discriminator's area over it, against the level either side), most
// of it at once. Indoors, echoes put dips in the envelope at certain brightness levels with no slip: those are
// picture, not clicks, and are left alone. They last as long as the picture stays at that brightness, and over a
// long dip the "area" is only how much brighter or darker that stretch is than its surroundings, times its
// length: any amount. So a dip counts only if
//  - it is short: noise clicks last 1-3 samples at 20 MS/s (the channel is ~18 MHz wide), a fade or a stretch
//    of picture at an echo's notch lasts longer;
//  - its area is near one turn, not merely above a part of one;
//  - it is abrupt: some sample of it is 8 MHz away from the level either side. A click's turn comes within a
//    sample or two (8-14 MHz at 20 MS/s). A picture edge crossing a notch, or a thin white stroke of OSD text
//    whose brightness sits on one, can pass the first two tests but stays under that: black to white is 5.3 MHz
//    at 8 MHz deviation, and the echo's own phase swing adds up to 2.5 when it is 3 dB under the direct signal.
//    (With a stronger echo, or over a strongly coloured background, such strokes can still be taken for clicks.
//    7 MHz finds 2 % more of the real clicks and takes two to three times as many strokes; 9 MHz loses a tenth
//    of the clicks.)
class ClickDetector {
public:
  explicit ClickDetector(double fs) : _fs(fs) {}
  // A call's samples begin. Off: nothing is kept from the calls before. On: the end of the last call's envelope
  // stays in front of this call's, so that a dip at the start of a call is seen whole.
  void begin(bool on);
  // The next n samples (re + j im), in order: their envelope is noted.
  void envelope(const float *re, const float *im, int n);
  // The call's samples are all in, d their discriminator output (MHz): bad becomes one byte per sample, 1 where
  // the carrier collapsed. With no samples since begin(), bad is left as it is.
  void mark(const std::vector<float> &d, std::vector<uint8_t> &bad);

private:
  double _fs;
  std::vector<float> _env;    // each sample's envelope (power), the kept end of the last call's in front
  std::vector<float> _dtail;  // the discriminator output of those kept samples
  size_t _hist = 0;           // how many are kept ones
  float _pref = 0;            // the envelope's recent level
};

// The decoder's half: the picture in the rings patched over the marks, line by line, and rows too badly hit for
// that redrawn from their neighbours.
class SparkleRepair {
public:
  explicit SparkleRepair(double fs);
  // The line whose picture is samples [pic0, pic0 + aw): each run of marked samples (widened by the filters'
  // spread) becomes a straight line between the points either side, in the filtered luma and chroma. Chroma is
  // patched further on than luma: its low-pass is narrower, and a patch that ends inside the click's own
  // response there draws a ramp to a wrong colour and leaves the rest as a coloured streak.
  // T: the line period in samples. luma_comb: the comb's look at the line before (R.comb) is patched too.
  // colour_fades: the adaptive comb is fading colour in the filter (it is on, without the 3-line comb).
  // Returns true when the line is still badly hit (over 8 % of it lost, or a run too long to patch): pass that
  // to mend_rows().
  bool patch_line(SampleRings &R, double pic0, double aw, double T, bool luma_comb, bool colour_fades);
  // Row k of a field (field: its rows of `width` RGBX pixels) has been drawn; hit: patch_line's answer for it.
  // Up to 3 badly hit rows in a row are redrawn from the good rows above and below them in the same field
  // (blended by distance), once the row below is in. Nothing older is used, so there is no tearing in motion; a
  // longer run is left as it is. field_no, which: tell one field from the next.
  void mend_rows(uint8_t *field, int width, uint64_t field_no, int which, int64_t k, bool hit);

  uint64_t clicks_repaired = 0, lines_mended = 0;

private:
  int _fwd_c, _fwd_low, _fwd_wide;  // samples a click lasts in chroma / in the comb's low and wide taps
  int _lobe0, _lobe1;  // chroma: where the click's main lobe is, in samples after a run's first mark
  int64_t _bad_from = -1;  // first of the badly hit rows waiting for a good row below
  uint64_t _mend_field = ~0ull;
  int _mend_which = -1;
};

}  // namespace kp
