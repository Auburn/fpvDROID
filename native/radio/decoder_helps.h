// fpvDROID radio: the decoder's helps as debug switches, to see what each does by itself. Each is a DecoderOpts flag
// (video.h) with a name: the app's "env" extra (Native.setDebugEnv) and kp_live's environment variable of that name.
// Each starts the way DecoderOpts has it (all on), and a change takes effect with the next decoder the radio makes
// (Radio::new_front_end: at the next carrier or rate watched).
// Adding one: a DecoderOpts field, a row in kDecoderHelps, and its sentence in docs/development.md.
#pragma once
#include <atomic>
#include <cstddef>
#include <iterator>
#include <string_view>

#include "video.h"

namespace kp {

struct DecoderHelp {
  const char *name;         // the debug switch
  bool DecoderOpts::*flag;  // what it turns on and off
};

inline constexpr DecoderHelp kDecoderHelps[] = {
    {"KP_LEVELS", &DecoderOpts::follow_levels},
    {"KP_FLYWHEEL", &DecoderOpts::flywheel},
    {"KP_HOLD", &DecoderOpts::hold},
    {"KP_FILTER", &DecoderOpts::channel_filter},
};

// The switches as last set: set() from any thread, fill() when a decoder is made.
class DecoderHelps {
public:
  DecoderHelps() {
    const DecoderOpts d;
    for (size_t i = 0; i < kCount; i++) _on[i].store(d.*kDecoderHelps[i].flag, std::memory_order_relaxed);
  }
  // Whether name is one of kDecoderHelps.
  static bool known(std::string_view name) { return index(name) < kCount; }
  // False: name is not one of them, and nothing changed.
  bool set(std::string_view name, bool on) {
    const size_t i = index(name);
    if (i == kCount) return false;
    _on[i].store(on, std::memory_order_relaxed);
    return true;
  }
  // Each help's flag in o as last set; o's other fields are left as they are.
  void fill(DecoderOpts &o) const {
    for (size_t i = 0; i < kCount; i++) o.*kDecoderHelps[i].flag = _on[i].load(std::memory_order_relaxed);
  }

private:
  static constexpr size_t kCount = std::size(kDecoderHelps);
  static size_t index(std::string_view name) {
    size_t i = 0;
    while (i < kCount && name != kDecoderHelps[i].name) i++;
    return i;
  }
  std::atomic<bool> _on[kCount];
};

}  // namespace kp
