// kepyur: the app's screen on Android (the Kotlin app's Surface; kepyur_jni.cpp hands it frames and settings).
// Frames go straight to the video Surface: a render thread owns an EGL context on it and draws each frame as
// it arrives (latest frame wins; the decode thread never waits for the display).
#pragma once
#include <EGL/egl.h>
#include <android/native_window.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "log.h"
#include "video.h"
#include "video_renderer.h"

namespace kp {

// One thread owns the EGL context and everything GL. Others hand it things under _m: a frame (post), a window
// (set_window, which waits until the old one has been let go), the view settings, the menu backdrop.
class Screen {
  using Clock = std::chrono::steady_clock;

public:
  Screen() : _th([this] { loop(); }) {}
  ~Screen() {
    {
      std::lock_guard<std::mutex> l(_m);
      _stop = true;
    }
    _cv.notify_all();
    _th.join();
  }

  // The Surface to draw on, or nullptr when it is going away. Takes the reference. Returns once the render
  // thread no longer touches the previous window: the app may destroy its Surface right after.
  void set_window(ANativeWindow *w) {
    std::unique_lock<std::mutex> l(_m);
    if (_win_next_set && _win_next) ANativeWindow_release(_win_next);  // never picked up
    _win_next = w;
    _win_next_set = true;
    const uint64_t want = ++_win_seq;
    _cv.notify_all();
    // a render thread that died (no GL at all) must not hang the UI thread
    _done.wait_for(l, std::chrono::milliseconds(1500), [&] { return _win_done >= want || _stop; });
  }

  // The picture stays for up to `ms` (dimmed once it is stale, not faded to black), and clear() is ignored meanwhile:
  // for the few seconds the adapter is opened again after its USB data path wedged. The next frame ends it.
  void hold(int ms) {
    {
      std::lock_guard<std::mutex> l(_m);
      _hold_until = Clock::now() + std::chrono::milliseconds(std::max(0, ms));
    }
    _cv.notify_one();
  }

  // Decode thread: copy and go.
  void post(const Frame &f) {
    {
      std::lock_guard<std::mutex> l(_m);
      if (Clock::now() < _quiet_until) return;
      _hold_until = {};
      _w = f.width;
      _h = f.height;
      _pending.assign(f.data, f.data + static_cast<size_t>(f.width) * f.height * 4);
      _have = true;
    }
    _cv.notify_one();
  }

  // No picture any more (the radio closed, another channel was asked for): black until the next frame. The
  // decoder may still have a frame of the old picture on its way: frames are refused for a moment (no new
  // picture can be locked that quickly). Not while a picture is held (hold()).
  void clear() {
    {
      std::lock_guard<std::mutex> l(_m);
      if (Clock::now() < _hold_until) return;
      _have = false;
      _clear = true;
      _quiet_until = Clock::now() + std::chrono::milliseconds(120);
    }
    _cv.notify_one();
  }

  // Layout (the blur and gain fields are the screen's own business).
  void set_view(const ViewOpts &v) {
    {
      std::lock_guard<std::mutex> l(_m);
      _view = v;
      _dirty = true;
    }
    _cv.notify_one();
  }

  // The menu's backdrop: the picture blurred and dimmed. Eased in and out unless `now`.
  void set_backdrop(bool on, bool now) {
    {
      std::lock_guard<std::mutex> l(_m);
      _blur_to = on ? 1.f : 0.f;
      if (now) _blur_snap = true;
      _dirty = true;
    }
    _cv.notify_one();
  }

  void redraw() {
    {
      std::lock_guard<std::mutex> l(_m);
      _dirty = true;
    }
    _cv.notify_one();
  }

private:
  static constexpr float kBlurSeconds = 0.26f;   // backdrop fade
  static constexpr double kStaleSeconds = 1.2;   // no frame for this long: the picture is not live any more ...
  static constexpr float kFadeSeconds = 0.5f;    // ... and fades to black over this
  static constexpr float kHoldGain = 0.6f;       // ... or only this far while held (hold())

  // EGL sorts the configs that match by colour depth, deepest first, so the first one is 10-bit wherever the
  // driver has such a config. The picture is 8-bit, and its buffer format then follows the phone, with a 2-bit
  // alpha the compositor may not put on an overlay. Of the configs that are exactly 8 bits a colour, take the
  // one with the least besides: no multisampling first (it costs the GPU on every frame), then opaque, then no
  // depth or stencil buffer (memory only; the emulator's EGL has no config without them). Only a driver with no
  // 8-bit config at all gets its first choice.
  static EGLConfig pick_config(EGLDisplay dpy) {
    const EGLint attr[] = {EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
                           EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_NONE};
    EGLint n = 0;
    if (!eglChooseConfig(dpy, attr, nullptr, 0, &n) || n < 1) return nullptr;
    std::vector<EGLConfig> all(n);
    if (!eglChooseConfig(dpy, attr, all.data(), n, &n) || n < 1) return nullptr;
    const auto get = [dpy](EGLConfig c, EGLint a) {
      EGLint v = -1;
      eglGetConfigAttrib(dpy, c, a, &v);
      return v;
    };
    EGLConfig best = all[0];
    int best_cost = 1 << 30;  // of an 8-bit config; lower is better, the driver's order decides between equals
    for (EGLint i = 0; i < n; i++) {
      const EGLConfig c = all[i];
      if (get(c, EGL_RED_SIZE) != 8 || get(c, EGL_GREEN_SIZE) != 8 || get(c, EGL_BLUE_SIZE) != 8) continue;
      const EGLint a = get(c, EGL_ALPHA_SIZE);
      const int cost = (get(c, EGL_SAMPLE_BUFFERS) != 0 || get(c, EGL_SAMPLES) > 0 ? 100 : 0) +
                       (a == 0 ? 0 : a == 8 ? 10 : 20) + (get(c, EGL_DEPTH_SIZE) != 0 ? 2 : 0) +
                       (get(c, EGL_STENCIL_SIZE) != 0 ? 1 : 0);
      if (cost < best_cost) best = c, best_cost = cost;
    }
    KP_LOGI("screen: EGL config R%d G%d B%d A%d, depth %d, stencil %d, samples %d, native visual 0x%x%s",
            get(best, EGL_RED_SIZE), get(best, EGL_GREEN_SIZE), get(best, EGL_BLUE_SIZE), get(best, EGL_ALPHA_SIZE),
            get(best, EGL_DEPTH_SIZE), get(best, EGL_STENCIL_SIZE), get(best, EGL_SAMPLES),
            get(best, EGL_NATIVE_VISUAL_ID), best_cost == 1 << 30 ? " (no 8-bit one: the driver's first)" : "");
    return best;
  }

  bool egl_up() {
    if (_dpy != EGL_NO_DISPLAY) return _ctx != EGL_NO_CONTEXT;
    _dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (_dpy == EGL_NO_DISPLAY || !eglInitialize(_dpy, nullptr, nullptr)) {
      KP_LOGE("screen: no EGL display (0x%x)", eglGetError());
      return false;
    }
    _cfg = pick_config(_dpy);
    if (!_cfg) {
      KP_LOGE("screen: no OpenGL ES 3 config (0x%x)", eglGetError());
      return false;
    }
    const EGLint ctx_attr[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
    _ctx = eglCreateContext(_dpy, _cfg, EGL_NO_CONTEXT, ctx_attr);
    if (_ctx == EGL_NO_CONTEXT) KP_LOGE("screen: no OpenGL ES 3 context (0x%x)", eglGetError());
    return _ctx != EGL_NO_CONTEXT;
  }

  void drop_surface() {
    if (_surf != EGL_NO_SURFACE) {
      eglMakeCurrent(_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
      eglDestroySurface(_dpy, _surf);
      _surf = EGL_NO_SURFACE;
    }
    if (_win) {
      ANativeWindow_release(_win);
      _win = nullptr;
    }
  }

  void take_window(ANativeWindow *w) {
    drop_surface();
    if (!w) return;
    _win = w;
    if (!egl_up()) return;
    EGLint format = 0;
    eglGetConfigAttrib(_dpy, _cfg, EGL_NATIVE_VISUAL_ID, &format);
    ANativeWindow_setBuffersGeometry(w, 0, 0, format);
    _surf = eglCreateWindowSurface(_dpy, _cfg, w, nullptr);
    if (_surf == EGL_NO_SURFACE || !eglMakeCurrent(_dpy, _surf, _surf, _ctx)) {
      KP_LOGE("screen: no window surface (0x%x)", eglGetError());
      drop_surface();
      return;
    }
    // Never wait for the display's refresh: a frame is drawn when it is decoded and the compositor shows the
    // newest one. Waiting would add up to a refresh period of delay and stall this thread behind it.
    eglSwapInterval(_dpy, 0);
    if (!_gl_ready) {
      _gl_ready = _gl.init();
      if (!_gl_ready) KP_LOGE("screen: renderer did not start");
      else KP_LOGI("screen: OpenGL ES renderer ready");
      _tex_stale = true;
    }
  }

  void loop() {
    std::vector<uint8_t> px, last;  // the frame in hand, and a copy of the newest (re-uploaded after a GL restart)
    int w = 0, h = 0, lw = 0, lh = 0;
    float blur = 0, gain = 0;
    auto t_prev = Clock::now(), t_frame = t_prev;
    bool live = false;  // a frame within kStaleSeconds
    for (;;) {
      bool got = false, cleared = false, animating;
      ViewOpts view;
      float blur_to;
      float floor;  // what a stale picture fades to: black, or kHoldGain while it is held (hold())
      {
        std::unique_lock<std::mutex> l(_m);
        const auto ready = [&] { return _stop || _win_next_set || _have || _clear || _dirty; };
        floor = Clock::now() < _hold_until ? kHoldGain : 0.f;
        animating = blur != _blur_to || (live ? gain < 1 : gain > floor);
        if (animating) _cv.wait_for(l, std::chrono::milliseconds(8), ready);
        else if (live) _cv.wait_until(l, t_frame + std::chrono::duration<double>(kStaleSeconds), ready);
        else if (floor > 0) _cv.wait_until(l, _hold_until, ready);  // then on to black, unless a frame came
        else _cv.wait(l, ready);
        if (_stop) break;
        floor = Clock::now() < _hold_until ? kHoldGain : 0.f;
        if (_win_next_set) {
          ANativeWindow *nw = _win_next;
          _win_next = nullptr;
          _win_next_set = false;
          const uint64_t seq = _win_seq;
          l.unlock();
          take_window(nw);
          l.lock();
          _win_done = seq;
          _done.notify_all();
        }
        if (_have) {
          px.swap(_pending);
          w = _w;
          h = _h;
          _have = false;
          got = true;
        }
        cleared = _clear;
        _clear = false;
        _dirty = false;
        view = _view;
        blur_to = _blur_to;
        if (_blur_snap) blur = blur_to;
        _blur_snap = false;
      }
      const auto now = Clock::now();
      // time since the last step of a fade in progress; a fade that starts now starts from here
      const float dt = animating ? std::min(std::chrono::duration<float>(now - t_prev).count(), 0.05f) : 0.f;
      t_prev = now;
      if (cleared && !got) {
        live = false;
        gain = 0;
        last.clear();
      }
      if (got) {
        t_frame = now;
        live = true;
      } else if (live && std::chrono::duration<double>(now - t_frame).count() >= kStaleSeconds) {
        live = false;
      }
      // ease the backdrop and the picture's fade toward where they should be
      const float bstep = dt / kBlurSeconds, gstep = dt / kFadeSeconds;
      blur = blur < blur_to ? std::min(blur_to, blur + bstep) : std::max(blur_to, blur - bstep);
      if (live) gain = std::min(1.f, gain + 4 * gstep);
      else if (gain > floor) gain = std::max(floor, gain - gstep);  // a stale picture is never brightened
      if (_surf == EGL_NO_SURFACE || !_gl_ready) {
        if (got) last.swap(px), lw = w, lh = h;
        continue;
      }
      if (got) {
        _gl.upload(px.data(), w, h);
        last.swap(px);
        lw = w, lh = h;
        _tex_stale = false;
      } else if (_tex_stale && !last.empty()) {
        _gl.upload(last.data(), lw, lh);
        _tex_stale = false;
      }
      if (cleared && !got) _gl.clear();
      EGLint sw = 0, sh = 0;
      eglQuerySurface(_dpy, _surf, EGL_WIDTH, &sw);
      eglQuerySurface(_dpy, _surf, EGL_HEIGHT, &sh);
      view.blur = blur * blur * (3 - 2 * blur);  // smoothstep
      view.gain = gain;
      _gl.draw(0, sw, sh, view);
      if (!eglSwapBuffers(_dpy, _surf)) {
        const EGLint e = eglGetError();
        KP_LOGW("screen: swap failed (0x%x)", e);
        if (e == EGL_CONTEXT_LOST) {  // the GPU was reset under us: everything GL is gone, start over
          _gl.destroy(false);
          _gl_ready = false;
          ANativeWindow *keep = _win;
          if (keep) ANativeWindow_acquire(keep);
          drop_surface();
          eglDestroyContext(_dpy, _ctx);
          _ctx = EGL_NO_CONTEXT;
          eglTerminate(_dpy);
          _dpy = EGL_NO_DISPLAY;
          take_window(keep);
        }
      }
    }
    if (_gl_ready && _surf != EGL_NO_SURFACE) _gl.destroy();
    drop_surface();
    if (_dpy != EGL_NO_DISPLAY) {
      if (_ctx != EGL_NO_CONTEXT) eglDestroyContext(_dpy, _ctx);
      eglTerminate(_dpy);
    }
    std::lock_guard<std::mutex> l(_m);
    if (_win_next) ANativeWindow_release(_win_next);
    _win_next = nullptr;
    _win_done = _win_seq;
    _done.notify_all();
  }

  std::mutex _m;
  std::condition_variable _cv, _done;
  // handed over under _m
  ANativeWindow *_win_next = nullptr;
  bool _win_next_set = false;
  uint64_t _win_seq = 0, _win_done = 0;
  std::vector<uint8_t> _pending;
  int _w = 0, _h = 0;
  bool _have = false, _clear = false, _dirty = false, _stop = false;
  std::chrono::steady_clock::time_point _quiet_until{};
  std::chrono::steady_clock::time_point _hold_until{};
  ViewOpts _view;
  float _blur_to = 0;
  bool _blur_snap = false;
  // render thread only
  EGLDisplay _dpy = EGL_NO_DISPLAY;
  EGLConfig _cfg = nullptr;
  EGLContext _ctx = EGL_NO_CONTEXT;
  EGLSurface _surf = EGL_NO_SURFACE;
  ANativeWindow *_win = nullptr;
  VideoRenderer _gl;
  bool _gl_ready = false, _tex_stale = false;
  std::thread _th;
};

}  // namespace kp
