// fpvDROID screen: front-buffer drawing, what Vsync off does where the phone can. The window surface is made
// single-buffered, so that what is drawn goes straight into the buffer the display shows, with no queue and no
// compositor frame in between (where the compositor gives the picture an overlay of its own). The screen then
// draws each strip as it comes, over its own part of the screen only (VideoRenderer::draw_band). The tearing that
// goes with it runs across the picture's lines (the panel scans along its long side).
//
// Measured on a Pixel 8 Pro at 120 Hz, by the app's own readout: the picture's top line is drawn 5 ms after the
// adapter took it, where through the compositor it is shown after 34 ms. It costs a quarter of a core more (a draw
// and a wait for the GPU for every strip).
//
// How: EGL_KHR_mutable_render_buffer makes the window surface single-buffered from the next swap (on Android: the
// window's buffer queue in shared-buffer mode), and EGL_ANDROID_front_buffer_auto_refresh has the compositor take
// that buffer at every refresh, queued or not.
#pragma once
#include <EGL/egl.h>

namespace kp {

// All of it on the thread that draws. Whether front-buffer drawing is asked for is the screen's to know.
class FrontBuffer {
public:
  // This EGL has the way to it (EGL_KHR_mutable_render_buffer) ...
  static bool available(EGLDisplay dpy);
  // ... with an EGL config whose window surfaces can be made single-buffered: the screen picks such a one at its
  // fresh start of EGL on the window.
  static bool config_can(EGLDisplay dpy, EGLConfig cfg);
  // On the surface just made (current in ctx, of config cfg). True: it is on, and the surface shows black. Where
  // the device cannot (no EGL_KHR_mutable_render_buffer, or the surface stays double-buffered), the log says so
  // and the picture goes up as without it.
  bool start(EGLDisplay dpy, EGLSurface surf, EGLContext ctx, EGLConfig cfg);
  // The surface is gone.
  void stop() { _on = false; }
  bool on() const { return _on; }
  // Returns once the GPU has done what it was given so far: then it is in the buffer the display shows. False: it
  // had not after timeout_ms.
  static bool wait_drawn(int timeout_ms);

private:
  bool _on = false;
};

}  // namespace kp
