// fpvDROID radio: demodulated blocks on their way from the DSP thread to the decode thread.
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "video.h"

namespace kp {

struct DemodBlock {
  std::shared_ptr<FrontEnd> fe;  // the decoder it is for
  std::vector<float> d;          // discriminator output
  std::vector<uint8_t> bad;      // sparkle repair marks (empty while it is off)
  bool gap = false;              // the stream lost samples before this block (a restart, a dropped bank)
  double t_sampled = 0;          // when its last sample was taken (steady clock, s; 0: not known)
};

// The DSP thread fills blocks and push()es them, the decode thread pop()s and decodes them. A decoder that falls
// behind loses the oldest block (it re-phases); the buffers of a block go round again (blank(), recycle()).
class DecodeQueue {
public:
  static constexpr size_t kCap = 128;   // blocks: ~26 ms behind at 40 MS/s
  static constexpr size_t kPool = 256;  // buffer pairs kept for reuse

  // DSP thread: a block to fill, with the buffers of an earlier one where there are any.
  DemodBlock blank();
  // DSP thread. A full queue loses its oldest block. That is not a hole for FrameGate to hide: a decoder that
  // falls behind loses blocks many times a second (a hot phone, a weak signal), and hiding a frame for each
  // would freeze the picture. A stream gap the lost block carried moves on to the block after it.
  void push(DemodBlock &&b);
  // Decode thread: the oldest block, waiting up to wait_ms for one (or for wake() with stop set). False: none.
  bool pop(DemodBlock &out, int wait_ms, const std::atomic<bool> &stop);
  // Decode thread: a decoded block's buffers, for blank().
  void recycle(DemodBlock &&b);
  void wake();   // a pop() that is waiting looks at `stop` again
  void clear();  // the blocks queued go (their buffers too)
  uint64_t dropped() const { return _dropped; }  // blocks lost to a full queue since reset_dropped()
  void reset_dropped() { _dropped = 0; }

private:
  struct Buffers {
    std::vector<float> d;
    std::vector<uint8_t> bad;
  };
  void keep(DemodBlock &b);  // under _m

  std::mutex _m;
  std::condition_variable _cv;
  std::deque<DemodBlock> _q;
  std::vector<Buffers> _pool;
  std::atomic<uint64_t> _dropped{0};
};

}  // namespace kp
