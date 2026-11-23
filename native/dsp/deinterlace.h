// fpvDROID decoder: fields into frames. A field is every other line of the picture: W x A pixels (RGBX), into a
// frame of W x 2A.
#pragma once
#include <cstdint>
#include <vector>

namespace kp {

// Field a on the frame's even lines and b on the odd ones (a frame woven from its two fields; the same field
// twice doubles its lines).
void weave(const uint8_t *a, const uint8_t *b, int W, int A, uint8_t *frame);

// Smooth lines (PictureOpts::smooth_lines): one field into the frame at its own lines (the even ones when upper), the
// lines between interpolated from the field lines above and below, along whichever of 5 directions matches best,
// so a diagonal edge stays a clean diagonal instead of a staircase.
class Deinterlacer {
public:
  void smooth(const uint8_t *field, bool upper, int W, int A, uint8_t *frame);

private:
  std::vector<uint16_t> _lum;  // each field row's brightness ...
  std::vector<int8_t> _dir;    // ... and one row's interpolation directions
};

}  // namespace kp
