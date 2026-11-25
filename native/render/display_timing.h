// fpvDROID screen: what the display says about time (EGL_ANDROID_get_frame_timestamps): when it showed each present,
// and the compositor's own timing: when it next composes, how often, and how long from there to the display.
#pragma once
#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <cstdint>

namespace kp {

// For the window surface in use, on the thread that draws on it.
class DisplayTiming {
public:
  // The surface just made (current): frame times on, where the display gives them (the log says which).
  void attach(EGLDisplay dpy, EGLSurface surf);
  // The surface is gone: nothing is known until the next.
  void detach();

  // The display says when each present went up (next_frame_id, shown_at).
  bool frame_times() const { return _frame_times; }
  // No presents are queued on this surface from here on (front-buffer drawing): nothing to give times of.
  void stop_frame_times() { _frame_times = false; }
  // The number of the next present (eglSwapBuffers), for shown_at; 0: none.
  uint64_t next_frame_id() const;
  // When present `id` went up on the display: ns on the steady clock, or one of LatencyMeter's codes (latency.h).
  int64_t shown_at(uint64_t id) const;

  // The compositor's next composition after t (steady clock, s): the deadline for a present to be in it, and the
  // compositor's own delay from composition to the display. It keeps these up to date with every present.
  // False: not known.
  bool composition_after(double t, double &deadline, double &to_display) const;
  // When the compositor expects to show a present handed over at t: its next composition, plus its delay to the
  // display. 0: not known.
  double expected_shown(double t) const;
  // The display's refresh rate as the compositor has it (how often it composes); 0: not known. Out of date while no
  // presents are queued: nothing then brings it up to date after a change of display mode.
  float refresh_hz() const;

private:
  EGLDisplay _dpy = EGL_NO_DISPLAY;
  EGLSurface _surf = EGL_NO_SURFACE;
  bool _frame_times = false;
  PFNEGLGETNEXTFRAMEIDANDROIDPROC _next_id = nullptr;
  PFNEGLGETFRAMETIMESTAMPSANDROIDPROC _frame_ts = nullptr;
  PFNEGLGETCOMPOSITORTIMINGANDROIDPROC _comp_timing = nullptr;
};

}  // namespace kp
