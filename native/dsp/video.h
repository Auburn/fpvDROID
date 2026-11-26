// fpvDROID: analog FPV video decoder (FM composite -> 8-bit grey frames). Shared by the app and the desktop
// tools. Grown from vtx-spike/viewer/vtxview.cpp.
//
//   FrontEnd fe(fs_in, shift_hz, on_frame);   // fs_in 40e6 (halfband to 20 MS/s) or 20e6
//   for (each IQ sample) fe.feed(x);
//
// Single-threaded: feed() and every callback run on the caller's thread.
#pragma once
#include <atomic>
#include <complex>
#include <cstdint>
#include <functional>
#include <vector>

namespace kp {

using cf = std::complex<float>;

struct VideoStd {
  double T_us;   // line period
  double a0_us;  // active video start after the hsync leading edge
  double aw_us;  // active video width
  int active;    // active lines per field
  int skip;      // lines from the field's first broad pulse to the first active line
  const char *name;
};
extern const VideoStd kNtsc, kPal;

// When the lines of a picture, or of a part of one, were taken: for telling how old they are on screen. On the
// steady clock, in seconds: when the adapter took the sample that was the decoder's newest as a line was drawn,
// reckoned from the time given with each block (FrontEnd::decode). 0: not known (no time was given, or none of the
// lines is drawn). first: the first line drawn of those in the part, last: the last.
struct LineTimes {
  double t_first = 0, t_last = 0;
};

// The picture, 4 bytes per pixel (R, G, B, 0xff), with its lines [y0, y1) new since the frame before. Valid only
// during the callback.
// Normally every frame is a finished picture with all of its lines new: [0, height). With low latency
// (PictureOpts::strips) a field goes out as several frames instead, each as soon as its lines are drawn: in order
// down the picture without gaps or overlaps, from line 0 to the bottom. The one that reaches the bottom finishes
// the picture (finished()); until then the lines from y1 down are still the picture before's.
struct Frame {
  const uint8_t *data;  // the whole picture, line 0 first
  int width, height;    // 720 x 480 (NTSC) / 720 x 576 (PAL)
  const VideoStd *std;
  int y0, y1;           // the lines that are new with this frame ...
  LineTimes times{};    // ... and when they were taken
  bool finished() const { return y1 >= height; }
};
using FrameSink = std::function<void(const Frame &)>;


struct DecoderStats {
  bool acquiring = true;
  float sign = 1, tip = 0, blank = 0;     // levels in MHz of discriminator output
  uint64_t lines = 0, lines_locked = 0;   // since the last acquisition
  uint64_t fields = 0, frames = 0;        // since construction
  uint64_t acquisitions = 0, losses = 0;  // successful acquisitions, lock losses
  uint64_t rephases = 0;                  // line grid moved onto a jumped signal without re-acquiring
  uint64_t level_jumps = 0;               // sync levels moved onto a signal that jumped up or down, likewise
  uint64_t clicks_repaired = 0;           // sparkle repair: FM clicks patched over ...
  uint64_t lines_mended = 0;              // ... and badly hit lines redrawn from the lines above and below
  double line_us = 0;                     // tracked line period
  float lock_recent = 0;                  // locked fraction over the last ~2 fields
  bool colour = false;                    // colour is being decoded (burst present and steady)
  float burst_err = 0;                    // mean burst phase error (rad)
  const char *std_name = "?";
};

struct DecoderOpts {
  int width = 720;
  bool fields = true;          // true: every field, line-doubled (60/50 Hz, no combing); false: woven frames
  int std_force = 0;           // 0 auto, 1 NTSC, 2 PAL
  bool force_invert = false;   // only try sync at the high-frequency end
  bool colour = true;          // decode colour (falls back to grey when there is no usable burst)
};

// The picture switches: one value, given to a FrontEnd when it is made and changed with set_picture(). A new set
// takes effect within a line of video, without rebuilding the decoder, and none of the switches holds the
// picture back (no added delay). All off by default. The first four are from Play RX
// (github.com/scerstt/PlayRX, GPL-2.0), where they were tuned on RTL8812BU adapters.
// One byte, so that a set is stored and read whole (std::atomic<PictureOpts>).
struct PictureOpts {
  // Luma comb (NTSC; PAL always uses the notch): how colour is kept out of the picture's brightness.
  //  off: a 3 MHz 4th-order low-pass + a notch at the subcarrier (rings ~4 px after every edge)
  //  on:  a gentle 4.8 MHz low-pass, colour cancelled with the line before (no ringing, sharper)
  bool luma_comb : 1 = false;
  // Adaptive comb (NTSC, with the luma comb on). The comb separates colour from brightness by comparing each
  // line with the one before; where that line is clearly different (OSD text, horizons) it leaves a halo and
  // false colour. There the brightness falls back to the notch filter and the colour is faded out. With the
  // chroma comb the colour is only faded where that comb finds no matching line below either.
  bool adaptive_comb : 1 = false;
  // Sparkle repair. The white / black sparkles of a weak signal are FM "clicks": the carrier's envelope
  // collapses for a moment and its phase slips a turn. The front end marks those instants (a longer dip of the
  // envelope, as an echo makes at one brightness, is not one); the decoder, which draws each line about a line
  // after it arrives, patches the picture over each mark (a straight line between the good points either side;
  // sync is left alone) and redraws a badly hit line from the lines above and below it.
  bool repair : 1 = false;
  // Smooth lines (DecoderOpts::fields). A field is every other line of the picture; shown with each line twice
  // it has stair-stepped edges, and the two fields sit half a line apart, so the picture shimmers up and down.
  // On: each field's lines go in their own place and the lines between are interpolated along edges (edge-based
  // line averaging), as each field ends.
  bool smooth_lines : 1 = false;
  // Chroma comb (NTSC): an adaptive 3-line comb for the colour: fewer rainbows on fine detail and OSD text, for
  // ~15 % more decode CPU. Each line is then drawn one line late, combed with the lines above and below it.
  bool chroma_comb : 1 = false;
  // Low latency (DecoderOpts::fields): each field goes out in strips, frames with about kStripRows of its lines new
  // (Frame::y0, y1), each as soon as those are drawn, instead of as one frame once the field has ended: the top of
  // the picture reaches the screen a field earlier. Lines sparkle repair may still redraw, and with smooth lines the
  // line below the newest one, wait for the next strip. Takes effect at the next field. Only for a sink that takes
  // a picture in parts (one that wants whole pictures would have to wait for Frame::finished()).
  // From the Play RX fork's line sink (github.com/scerstt/PlayRX).
  bool strips : 1 = false;
};

// Low latency: how many field lines a strip (PictureOpts::strips) holds: ~2 ms of picture, 8 strips a field NTSC,
// 9 PAL.
constexpr int kStripRows = 32;

class Decoder;
class ClickDetector;

// IQ (at fs_in) -> DC removal -> NCO -> [halfband /2 when fs_in >= 40e6] -> FM discriminator -> Decoder.
class FrontEnd {
public:
  FrontEnd(double fs_in, double shift_hz, FrameSink sink, const DecoderOpts &o = {}, const PictureOpts &p = {});
  ~FrontEnd();
  void set_shift(double shift_hz);  // carrier offset from the stream centre (Hz) to bring to 0
  double shift() const { return _shift; }
  // Any thread. demod() takes the new set at its next call, decode() at the next line of video.
  void set_picture(const PictureOpts &p) { _picture.store(p, std::memory_order_relaxed); }
  void feed(const cf *x, size_t n);  // demod + decode
  // The two halves, for running them on two threads: demod() -> discriminator output (MHz at fs_demod),
  // decode() -> frames. Each half is single-threaded.
  // bad (sparkle repair): per output sample, 1 = the carrier collapsed there (a click). demod() leaves it empty
  // while sparkle repair is off, and decode() takes an empty one as no marks.
  // t_end: when the adapter took the last sample of d (steady clock, s; 0: not known), for the frames' LineTimes.
  void demod(const cf *x, size_t n, std::vector<float> &out, std::vector<uint8_t> &bad);
  void decode(const std::vector<float> &d, const std::vector<uint8_t> &bad, double t_end = 0);
  DecoderStats stats() const;
  double fs_demod() const { return _fs_out; }

private:
  double _fs_in, _fs_out, _shift = 0;
  bool _decimate;
  static constexpr int kHb = 23;   // halfband taps
  static constexpr int kTab = 4096;  // mixer table / chunk length
  float _h[kHb];
  std::vector<float> _tr, _ti;     // step^i, i < kTab
  cf _stepN{1, 0}, _rot{1, 0}, _dc{}, _prev{1, 0};
  std::vector<float> _wr, _wi;     // mixed samples with kHb-1 of history in front
  std::vector<float> _yr, _yi;     // decimated
  int _phase = 0;                  // start of the next halfband window in the next chunk (0 or 1)
  std::vector<float> _dbuf;
  std::vector<uint8_t> _badbuf;
  std::atomic<PictureOpts> _picture;  // set_picture(): read by demod() at each call, by the decoder at each line
  Decoder *_dec;
  ClickDetector *_clicks;  // sparkle repair's half of demod() (sparkle.h)
};

}  // namespace kp
