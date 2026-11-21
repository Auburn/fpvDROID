// kepyur decoder: what the picture filters make of the discriminator output, kept for the last kSize samples.
// The decoder draws each line about a line after it arrives (and combs it with the lines either side), so
// everything that reads a line reads it from here, by absolute sample index: ring[i & mask].
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>
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

}  // namespace kp
