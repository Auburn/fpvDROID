// fpvDROID radio: when the adapter took the samples the decoder draws a picture from (LineTimes, video.h).
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace kp {

// The decode thread notes each block it is about to decode: where the block starts in the decoder's count of
// samples, how many samples it has, and when the adapter took its last one. Then the time of any sample in the last
// kBlocks blocks follows from the sample rate; one further back is reckoned from the oldest block kept, as if no
// samples had been lost since (it is off by any that were). One thread only.
class SampleClock {
public:
  static constexpr size_t kBlocks = 1024;  // ~200 ms at 20 MS/s in blocks of 4096: a whole field and more

  // A new decoder, at fs: nothing known.
  void reset(double fs);
  // The next block: the decoder's samples [first, first + n), the last one taken at t_last (steady clock, s; <= 0:
  // not known). Blocks come in order.
  void block(uint64_t first, size_t n, double t_last);
  // When sample at - 1 was taken (LineTimes counts the samples up to and including the one that completed a line);
  // 0 when that is not known: at 0, no blocks yet, or the block it falls in had no time.
  double sampled_at(uint64_t at) const;

private:
  struct Block {
    uint64_t first = 0, n = 0;
    double t_last = 0;
  };
  // the time of sample s, reckoned from block b (s may lie outside it)
  double from(const Block &b, uint64_t s) const;

  std::vector<Block> _ring = std::vector<Block>(kBlocks);
  size_t _next = 0, _count = 0;  // where the next block goes; how many are kept
  double _fs = 0;
};

}  // namespace kp
