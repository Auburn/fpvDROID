#include "thread_placement.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include "log.h"

namespace kp {

void boost_thread(const char *what) {
  const int tid = static_cast<int>(gettid());
  const int rc = setpriority(PRIO_PROCESS, tid, -10);
  KP_LOGI("radio: %s thread %d nice -10: %s", what, tid, rc == 0 ? "ok" : "refused");
}

#if defined(__linux__)
namespace {
// The cores a processing thread is kept on, by each core's cpu_capacity, else its top clock. Empty: no slower
// and faster kinds, fewer than two of the faster, or nothing readable.
std::vector<int> fast_cores() {
  const auto read = [](int c, const char *leaf) {
    char path[96];
    std::snprintf(path, sizeof path, "/sys/devices/system/cpu/cpu%d/%s", c, leaf);
    long v = -1;
    if (FILE *f = std::fopen(path, "r")) {
      if (std::fscanf(f, "%ld", &v) != 1) v = -1;
      std::fclose(f);
    }
    return v;
  };
  const int n = static_cast<int>(sysconf(_SC_NPROCESSORS_CONF));
  std::vector<long> cap(std::max(n, 0));
  for (const char *leaf : {"cpu_capacity", "cpufreq/cpuinfo_max_freq"}) {
    bool all = n > 0;
    for (int c = 0; c < n && all; c++) all = (cap[c] = read(c, leaf)) > 0;
    if (all) break;
    cap.assign(cap.size(), -1);
  }
  if (cap.empty() || cap[0] <= 0) return {};
  const long slowest = *std::min_element(cap.begin(), cap.end());
  std::vector<int> out;
  for (int c = 0; c < n; c++)
    if (cap[c] > slowest) out.push_back(c);
  if (out.size() < 2) out.clear();
  return out;
}
}  // namespace
#endif

void keep_off_slow_cores(const char *what, bool on) {
#if defined(__linux__)
  if (!on) {
    KP_LOGI("radio: %s thread left to the OS (KP_FAST_CPUS=0)", what);
    return;
  }
  static const std::vector<int> cores = fast_cores();
  if (cores.empty()) return;
  cpu_set_t set;
  CPU_ZERO(&set);
  std::string list;
  for (int c : cores) {
    CPU_SET(c, &set);
    list += (list.empty() ? "" : ",") + std::to_string(c);
  }
  const int rc = sched_setaffinity(0, sizeof set, &set);
  KP_LOGI("radio: %s thread on cores %s: %s", what, list.c_str(), rc == 0 ? "ok" : std::strerror(errno));
#else
  (void)what;
  (void)on;
#endif
}

}  // namespace kp
