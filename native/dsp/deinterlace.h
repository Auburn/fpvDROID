// fpvDROID decoder: fields into frames. A field is every other line of the picture: W x A pixels (RGBX), into a
// frame of W x 2A.
#pragma once
#include <cstdint>
#include <vector>

namespace kp {

// Field a on the frame's even lines and b on the odd ones (a frame woven from its two fields; the same field
// twice doubles its lines).
void weave(const uint8_t *a, const uint8_t *b, int W, int A, uint8_t *frame);

// Lines [y0, y1) of the frame made from one field with each of its lines twice (weave(f, f, ...) over those lines
// alone): frame line y is field row y / 2. frame is the whole frame (line 0 first).
void double_lines(const uint8_t *f, int W, int A, uint8_t *frame, int y0, int y1);

// Smooth lines (PictureOpts::smooth_lines): one field into the frame at its own lines (the even ones when upper), the
// lines between interpolated from the field lines above and below, along whichever of 5 directions matches best,
// so a diagonal edge stays a clean diagonal instead of a staircase.
class Deinterlacer {
public:
  // Lines [y0, y1) of the frame (y1 < 0: to the bottom); frame is the whole frame (line 0 first). A line between
  // field rows reads the rows either side of it: line y reads rows up to y / 2 + 1 (upper) or (y + 1) / 2 (lower), so
  // a field still being drawn can go out in parts (PictureOpts::strips), each the same as in the whole frame.
  void smooth(const uint8_t *field, bool upper, int W, int A, uint8_t *frame, int y0 = 0, int y1 = -1);

private:
  std::vector<uint16_t> _lum;  // each field row's brightness (those the lines asked for read) ...
  std::vector<int8_t> _dir;    // ... and one row's interpolation directions
};

}  // namespace kp
