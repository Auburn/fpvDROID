// fpvDROID decoder: what the picture filters make of the discriminator output, kept for the last kSize samples.
// The decoder draws each line about a line after it arrives (and combs it with the lines either side), so
// everything that reads a line reads it from here, by absolute sample index: ring[i & mask].
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace kp {

struct CombTap {
  float wide, low, d;  // wide luma, its low band (adaptive comb only), the raw discriminator output
};

struct SampleRings {
  static constexpr size_t kSize = 8192;  // 6.4 lines at 20 MS/s
  static constexpr size_t mask = kSize - 1;
  uint64_t n = 0;  // samples written so far: [n - kSize, n) are held
  std::vector<float> luma;                  // brightness, MHz of discriminator output
  std::vector<std::complex<float>> chroma;  // colour at baseband (U/V before the burst rotation)
  std::vector<CombTap> comb;                // luma comb: one array, so a look at the line before costs one miss
  std::vector<float> fade;                  // adaptive comb with the 3-line chroma comb: each sample's fade
  std::vector<uint8_t> bad;                 // sparkle repair: the front end's click marks

  SampleRings() : luma(kSize, 0), chroma(kSize), comb(kSize, CombTap{}), fade(kSize, 0), bad(kSize, 0) {}

  // Sample i has been written.
  bool written(int64_t i) const { return i < static_cast<int64_t>(n); }
  // Sample i has not been written over, nor will it be within the next `slack` samples.
  bool kept(int64_t i, int64_t slack = 0) const {
    return static_cast<int64_t>(n) - i <= static_cast<int64_t>(mask) - slack;
  }
  // Samples from..to are all there to read.
  bool has(int64_t from, int64_t to, int64_t slack = 0) const { return written(to) && kept(from, slack); }
};

// The sync path's samples (the decoder's 1 MHz low-pass and 3-point median), the last kMask + 1 of them by
// absolute index: for what looks at a line's sync once the line has ended (the fold of the lines: line_hold.h; the
// vertical sync by its half lines: vert_sync.h; the noise on the sync tip and the back porch: sync_noise.h).
struct SyncRing {
  static constexpr size_t kMask = 8191;
  std::vector<float> v;
  SyncRing() : v(kMask + 1, 0.f) {}
  // cnt of them (no more than the ring holds), the first being sample `at`: one stretch, or two where it wraps
  void put(uint64_t at, const float *m, size_t cnt) {
    const size_t i = static_cast<size_t>(at) & kMask, first = cnt < kMask + 1 - i ? cnt : kMask + 1 - i;
    std::memcpy(v.data() + i, m, first * sizeof(float));
    std::memcpy(v.data(), m + first, (cnt - first) * sizeof(float));
  }
};

// ... as the pulse tracking has them: it is behind what has been put (a piece of samples goes into the ring at
// once), and has taken n. What it has not had yet is not to be read.
struct SyncSamples {
  const SyncRing &ring;
  uint64_t n;
  float operator[](int64_t i) const { return ring.v[static_cast<size_t>(i) & SyncRing::kMask]; }
  // Sample i0 has not been written over.
  bool kept(int64_t i0) const { return i0 >= 0 && static_cast<int64_t>(n) - i0 <= static_cast<int64_t>(SyncRing::kMask); }
  // Samples [i0, i1) are all there to read.
  bool has(int64_t i0, int64_t i1) const { return kept(i0) && i1 <= static_cast<int64_t>(n); }
};

}  // namespace kp
