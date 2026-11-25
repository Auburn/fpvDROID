// fpvDROID screen: the display's timing. See display_timing.h.
#include "display_timing.h"

#include <cmath>

#include "egl_ext.h"
#include "latency.h"
#include "log.h"

namespace kp {

void DisplayTiming::attach(EGLDisplay dpy, EGLSurface surf) {
  detach();
  if (!egl_has(dpy, "EGL_ANDROID_get_frame_timestamps")) {
    KP_LOGI("screen: the display gives no frame times (no EGL_ANDROID_get_frame_timestamps)");
    return;
  }
  _dpy = dpy;
  _surf = surf;
  _next_id = reinterpret_cast<PFNEGLGETNEXTFRAMEIDANDROIDPROC>(eglGetProcAddress("eglGetNextFrameIdANDROID"));
  _frame_ts = reinterpret_cast<PFNEGLGETFRAMETIMESTAMPSANDROIDPROC>(eglGetProcAddress("eglGetFrameTimestampsANDROID"));
  _comp_timing =
      reinterpret_cast<PFNEGLGETCOMPOSITORTIMINGANDROIDPROC>(eglGetProcAddress("eglGetCompositorTimingANDROID"));
  const auto supported = reinterpret_cast<PFNEGLGETFRAMETIMESTAMPSUPPORTEDANDROIDPROC>(
      eglGetProcAddress("eglGetFrameTimestampSupportedANDROID"));
  if (!_next_id || !_frame_ts || !supported || !eglSurfaceAttrib(dpy, surf, EGL_TIMESTAMPS_ANDROID, EGL_TRUE) ||
      !supported(dpy, surf, EGL_DISPLAY_PRESENT_TIME_ANDROID)) {
    KP_LOGI("screen: the display gives no present times (0x%x)", eglGetError());
    return;
  }
  _frame_times = true;
  KP_LOGI("screen: frame times on (EGL_ANDROID_get_frame_timestamps)");
}

void DisplayTiming::detach() {
  _dpy = EGL_NO_DISPLAY;
  _surf = EGL_NO_SURFACE;
  _frame_times = false;
  _next_id = nullptr;
  _frame_ts = nullptr;
  _comp_timing = nullptr;
}

uint64_t DisplayTiming::next_frame_id() const {
  EGLuint64KHR id = 0;
  return _frame_times && _next_id(_dpy, _surf, &id) ? id : 0;
}

int64_t DisplayTiming::shown_at(uint64_t id) const {
  const EGLint what = EGL_DISPLAY_PRESENT_TIME_ANDROID;
  EGLnsecsANDROID v = 0;
  if (!_frame_times || !_frame_ts(_dpy, _surf, id, 1, &what, &v)) return LatencyMeter::kUnknown;
  if (v == EGL_TIMESTAMP_PENDING_ANDROID) return LatencyMeter::kPending;
  if (v == EGL_TIMESTAMP_INVALID_ANDROID) return LatencyMeter::kDropped;
  return v;
}

bool DisplayTiming::composition_after(double t, double &deadline, double &to_display) const {
  if (!_comp_timing) return false;
  const EGLint what[3] = {EGL_COMPOSITE_DEADLINE_ANDROID, EGL_COMPOSITE_INTERVAL_ANDROID,
                          EGL_COMPOSITE_TO_PRESENT_LATENCY_ANDROID};
  EGLnsecsANDROID v[3] = {};
  if (!_comp_timing(_dpy, _surf, 3, what, v) || v[0] <= 0 || v[1] <= 0 || v[2] <= 0) return false;
  deadline = static_cast<double>(v[0]) * 1e-9;
  const double interval = static_cast<double>(v[1]) * 1e-9;
  if (deadline < t) deadline += std::ceil((t - deadline) / interval) * interval;  // the first one after t
  to_display = static_cast<double>(v[2]) * 1e-9;
  return true;
}

double DisplayTiming::expected_shown(double t) const {
  double deadline, to_display;
  return composition_after(t, deadline, to_display) ? deadline + to_display : 0;
}

float DisplayTiming::refresh_hz() const {
  const EGLint what = EGL_COMPOSITE_INTERVAL_ANDROID;
  EGLnsecsANDROID v = 0;
  return _comp_timing && _comp_timing(_dpy, _surf, 1, &what, &v) && v > 0 ? static_cast<float>(1e9 / v) : 0.f;
}

}  // namespace kp
