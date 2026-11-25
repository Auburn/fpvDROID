#include "sample_clock.h"

namespace kp {

void SampleClock::reset(double fs) {
  _fs = fs;
  _next = _count = 0;
}

void SampleClock::block(uint64_t first, size_t n, double t_last) {
  if (n == 0) return;
  _ring[_next] = Block{first, n, t_last};
  _next = (_next + 1) % kBlocks;
  if (_count < kBlocks) _count++;
}

double SampleClock::from(const Block &b, uint64_t s) const {
  if (b.t_last <= 0) return 0;
  const double back = static_cast<double>(b.first + b.n - 1) - static_cast<double>(s);  // samples after s, to its end
  return b.t_last - back / _fs;
}

double SampleClock::sampled_at(uint64_t at) const {
  if (at == 0 || _count == 0 || _fs <= 0) return 0;
  const uint64_t s = at - 1;
  // newest first (the samples asked about are mostly from the last few blocks): the block s is in, or the last one
  // before it (s past the newest block, or in samples lost between two)
  for (size_t i = 0; i < _count; i++) {
    const Block &b = _ring[(_next + kBlocks - 1 - i) % kBlocks];
    if (s >= b.first) return from(b, s);
  }
  return from(_ring[(_next + kBlocks - _count) % kBlocks], s);  // before the oldest kept
}

}  // namespace kp
