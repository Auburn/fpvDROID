// fpvDROID decoder: fields into frames. See deinterlace.h.
#include "deinterlace.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

namespace kp {

void weave(const uint8_t *a, const uint8_t *b, int W, int A, uint8_t *frame) {
  const size_t rowb = static_cast<size_t>(W) * 4;
  for (int r = 0; r < A; r++) {
    std::memcpy(frame + (2 * r) * rowb, a + r * rowb, rowb);
    std::memcpy(frame + (2 * r + 1) * rowb, b + r * rowb, rowb);
  }
}

void double_lines(const uint8_t *f, int W, int A, uint8_t *frame, int y0, int y1) {
  const size_t rowb = static_cast<size_t>(W) * 4;
  y0 = std::max(y0, 0);
  y1 = std::min(y1, 2 * A);
  for (int y = y0; y < y1; y++) std::memcpy(frame + y * rowb, f + (y >> 1) * rowb, rowb);
}

void Deinterlacer::smooth(const uint8_t *f, bool upper, int W, int A, uint8_t *out, int y0, int y1) {
  const size_t rowb = static_cast<size_t>(W) * 4;
  y0 = std::max(y0, 0);
  y1 = std::min(y1, 2 * A);
  if (y0 >= y1 || A <= 0) return;
  const int own = upper ? 0 : 1;  // the parity of the frame lines that are the field's own rows
  for (int y = y0 + ((y0 & 1) != own); y < y1; y += 2) std::memcpy(out + y * rowb, f + (y >> 1) * rowb, rowb);
  // the brightness of each row the lines read, once: the search below compares 5 pairs of them per pixel
  const int r_lo = std::max(0, (y0 >> 1) - 1), r_hi = std::min(A - 1, ((y1 - 1) >> 1) + 1);
  _lum.resize(static_cast<size_t>(W) * A);
  for (size_t i = static_cast<size_t>(r_lo) * W, e = static_cast<size_t>(r_hi + 1) * W; i < e; i++) {
    const uint8_t *px = f + i * 4;
    _lum[i] = static_cast<uint16_t>(px[0] + 2 * px[1] + px[2]);
  }
  _dir.resize(W);
  const auto blend = [](const uint8_t *a, const uint8_t *b, uint8_t *dst) {  // per byte (a + b + 1) / 2
    uint32_t x, y;
    std::memcpy(&x, a, 4);
    std::memcpy(&y, b, 4);
    const uint32_t z = (x | y) - (((x ^ y) >> 1) & 0x7f7f7f7fu);
    std::memcpy(dst, &z, 4);
  };
  for (int line = y0 + ((y0 & 1) == own); line < y1; line += 2) {
    // the frame line between field rows rp (above it) and rq (below it); at the frame's edge, one row twice
    const int r = line >> 1;
    const int rp = upper ? r : std::max(r - 1, 0), rq = upper ? std::min(r + 1, A - 1) : r;
    const uint8_t *p = f + rp * rowb, *q = f + rq * rowb;
    const uint16_t *lp = _lum.data() + static_cast<size_t>(rp) * W, *lq = _lum.data() + static_cast<size_t>(rq) * W;
    uint8_t *dst = out + line * rowb;
    int8_t *best = _dir.data();
    // The direction d (p at x + d against q at x - d) with the lowest cost, straight down preferred; ties go
    // to the first of -2, -1, 0, 1, 2. Away from the row's ends all five exist: no branches, so it vectorises.
    const auto edge = [&](int x) {
      int b = 0, bc = 1 << 30;
      for (int d = std::max(-2, std::max(-x, x - (W - 1))), de = std::min(2, std::min(x, W - 1 - x)); d <= de; d++) {
        const int c = std::abs(lp[x + d] - lq[x - d]) + (d ? 24 + 8 * std::abs(d) : 0);
        if (c < bc) bc = c, b = d;
      }
      best[x] = static_cast<int8_t>(b);
    };
    for (int x = 0; x < std::min(2, W); x++) edge(x);
    for (int x = 2; x < W - 2; x++) {
      int bc = std::abs(lp[x - 2] - lq[x + 2]) + 40, b = -2;
      const int c1 = std::abs(lp[x - 1] - lq[x + 1]) + 32, c2 = std::abs(lp[x] - lq[x]);
      const int c3 = std::abs(lp[x + 1] - lq[x - 1]) + 32, c4 = std::abs(lp[x + 2] - lq[x - 2]) + 40;
      b = c1 < bc ? -1 : b, bc = std::min(bc, c1);
      b = c2 < bc ? 0 : b, bc = std::min(bc, c2);
      b = c3 < bc ? 1 : b, bc = std::min(bc, c3);
      b = c4 < bc ? 2 : b;
      best[x] = static_cast<int8_t>(b);
    }
    for (int x = std::max(2, W - 2); x < W; x++) edge(x);
    for (int x = 0; x < W; x++) blend(p + (x + best[x]) * 4, q + (x - best[x]) * 4, dst + x * 4);
  }
}

}  // namespace kp
