// fpvDROID: the two analog standards the decoder knows, and what it keeps of each line and field.
#pragma once

namespace kp {

// The part of each field that the decoder draws, which is also the frame's size: the whole picture, with a sliver of
// blanking round it.
struct PictureWindow {
  double start_us;  // its left edge after the hsync's leading edge ...
  double width_us;  // ... and its width
  int first_line;   // its top line, counted from the line of the field's first broad pulse ...
  int lines;        // ... and how many lines it takes from each field
  constexpr int frame_lines() const { return 2 * lines; }  // a frame's height: both fields' lines
};

struct VideoStd {
  double T_us;      // line period
  int field_lines;  // whole lines in an odd field; an even one has one more
  double fsc;       // colour subcarrier, Hz
  PictureWindow window;
  const char *name;
};

// Across, the window is BT.601's line (720 samples at 13.5 MHz, from `lead` samples after the hsync: 122 for 525
// lines, 132 for 625) and a sliver either side. That line is what an FPV camera and its OSD fill. An OSD chip of
// the MAX7456 kind draws 30 characters of 12 pixels at 6.75 MHz, which is those 720 samples. The standards'
// narrower active line (NTSC 9.4-62.0 us, PAL 10.5-62.5 us, which this decoder used to keep) cut about a third
// off the outer characters on both sides.
// Measured on the recordings in logs/ (not in the repo):
//  - NTSC (strips/lit): the camera's picture 8.85-62.3 us, the OSD 9.0-62.3 us; BT.601's line is 9.04-62.37 us.
//  - PAL (pixel7a, pixel8: one quad's OSD on its chip's own grey, with no camera): the grey (the OSD's 30 columns)
//    9.65-62.85 us, the text 10.0-62.45 us; BT.601's line is 9.78-63.11 us.
// The sliver ahead keeps the camera's first pixels and begins after the colour burst. The one after ends ahead of
// the next hsync and of the low-pass's ringing before it.
constexpr PictureWindow bt601_window(int lead, int first_line, int lines) {
  constexpr double kAheadUs = 0.49, kAfterUs = 0.28;
  return {lead / 13.5 - kAheadUs, 720 / 13.5 + kAheadUs + kAfterUs, first_line, lines};
}

// Down: NTSC's picture lines were 19-258 and 20-259 (by field) after the line of the first broad pulse, and line
// 260 is the next vertical sync's equalizing pulses in both fields, so lines 18-259 are kept: 720 x 484, the
// whole picture shown at 4:3. A camera whose 4:3 picture fills 53.45 us and 240 lines keeps its shape to within
// 0.5 %; the standard's window stretched it 1.6 % wide.
// PAL keeps the standard's lines, 22-309 (line 310 is equalizing pulses in one of its two fields; the OSD's
// text ran on lines 26-251). Being wider and no taller, it shows a PAL camera's 4:3 picture (52-53.3 us wide)
// 1.4-3.9 % narrow. That is unmeasured, because the PAL recordings have no camera.
inline constexpr VideoStd kNtsc{63.5556, 262, 315e6 / 88, bt601_window(122, 18, 242), "NTSC"};
inline constexpr VideoStd kPal{64.0, 312, 4.43361875e6, bt601_window(132, 22, 288), "PAL"};

}  // namespace kp
