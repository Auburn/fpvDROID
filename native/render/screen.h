// fpvDROID: the app's screen on Android (the Kotlin app's Surface; fpvdroid_jni.cpp hands it frames and settings).
// Frames go straight to the video Surface: a render thread owns an EGL context on it and draws each frame as
// it arrives (latest frame wins; the decode thread never waits for the display). With low latency
// (PictureOpts::strips) the picture comes in strips instead, and goes up once a refresh, just before the compositor
// takes its next frame, with the strips that have come by then: only the rows that changed go to the GPU.
//
// Front-buffer drawing (a prototype, set_front_buffer): the window surface single-buffered, so that what is drawn
// goes straight into the buffer the display shows, with no queue and no compositor frame in between (where the
// compositor gives the picture an overlay of its own); each strip is drawn as it comes, over its own part of the
// screen only. The tearing that goes with it runs across the picture's lines (the panel scans along its long side).
#pragma once
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <android/hardware_buffer.h>
#include <android/native_window.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "latency.h"
#include "log.h"
#include "video.h"
#include "video_renderer.h"

namespace kp {

// One thread owns the EGL context and everything GL. Others hand it things under _m: a frame (post), a window
// (set_window, which waits until the old one has been let go), the view settings, the menu backdrop, where the
// menu's preview is and what the card says where there is no picture.
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

  // The picture stays for up to `ms` (dimmed once it is stale, not faded into NO SIGNAL), and clear() is ignored meanwhile:
  // for the few seconds the adapter is opened again after its USB data path wedged. The next frame ends it.
  void hold(int ms) {
    {
      std::lock_guard<std::mutex> l(_m);
      _hold_until = Clock::now() + std::chrono::milliseconds(std::max(0, ms));
    }
    _cv.notify_one();
  }

  // Decode thread: copy and go. A frame whose strips came already (low latency) is in.
  void post(const Frame &f) {
    if (!f.in_strips) put(f.data, f.width, f.height, 0, f.height, f.times.t_first, f.times.t_last, f.times.t_last);
  }

  // Decode thread: a strip of the picture (low latency), into the picture as it was; up with the next present.
  void post_rows(const Strip &s) {
    put(s.data, s.width, s.height, s.y0, s.rows, s.y0 == 0 ? s.times.t_first : 0, s.last ? s.times.t_last : 0,
        s.times.t_last, true);
  }

  // Debug (KP_PRESENT_ALIGN, KP_PRESENT_MARGIN_US): strips go up once a refresh, margin_us before the compositor's
  // next composition (align, the default), or each as it comes (not align). Measured on a Pixel 8 Pro at 120 Hz: a
  // present for every strip (~350 a second) made the compositor show each frame a refresh later than one a refresh
  // did, and the display's history of frame times was too short to say when strips went up.
  void set_present(bool align, int margin_us) {
    _align = align;
    _margin_us = std::clamp(margin_us, 0, 20000);
    _cv.notify_one();
  }

  // Debug (KP_FRONT_BUFFER): front-buffer drawing (see the top of this file), from a fresh start of EGL on the
  // window (it may need another config). Where the device cannot (no EGL_KHR_mutable_render_buffer, or the surface
  // stays double-buffered), the log says so and the picture goes up as without it.
  void set_front_buffer(bool on) {
    {
      std::lock_guard<std::mutex> l(_m);
      if (_front_req == on) return;
      _front_req = on;
      _rebuild = true;
    }
    _cv.notify_one();
  }

  // The display's refresh rate as the app sees it (its Display's mode), each time that changes. The readout takes
  // it where the compositor's own timing is out of date: with front-buffer drawing no frames are queued, and
  // nothing brings that timing up to date after a mode change (on a Pixel 8 Pro it said 120 Hz after the panel
  // went to 60).
  void set_display_hz(float hz) { _display_hz = hz > 0 ? hz : 0.f; }

  // The picture's delay over the last second or so (latency.h), as JSON for the app.
  std::string latency_json() {
    LatencyStats s;
    bool front;
    {
      std::lock_guard<std::mutex> l(_m);
      s = _lat_shown;
      front = _front_shown;
    }
    char b[400];
    std::snprintf(b, sizeof b,
                  "{\"presents_s\":%.0f,\"shown_s\":%.0f,\"queued_top_ms\":%.1f,\"queued_bottom_ms\":%.1f,"
                  "\"shown_top_ms\":%.1f,\"shown_bottom_ms\":%.1f,\"shown_top_p95_ms\":%.1f,"
                  "\"expected_top_ms\":%.1f,\"expected_bottom_ms\":%.1f,\"refresh_hz\":%.1f,\"front\":%s}",
                  s.seconds > 0 ? s.presents / s.seconds : 0.f, s.seconds > 0 ? s.shown / s.seconds : 0.f,
                  s.queued_top, s.queued_bottom, s.shown_top, s.shown_bottom, s.shown_top_p95, s.expected_top,
                  s.expected_bottom, s.refresh_hz, front ? "true" : "false");
    return b;
  }

  // No picture any more (the radio closed, another channel was asked for): NO SIGNAL until the next frame. The
  // decoder may still have a frame of the old picture on its way: frames are refused for a moment (no new
  // picture can be locked that quickly). Not while a picture is held (hold()).
  void clear() {
    {
      std::lock_guard<std::mutex> l(_m);
      if (Clock::now() < _hold_until) return;
      _have = false;
      _pending.clear();  // the next picture's strips do not land on this one
      _w = _h = 0;
      _clear = true;
      _quiet_until = Clock::now() + std::chrono::milliseconds(120);
    }
    _cv.notify_one();
  }

  // The view settings: how the picture is laid out.
  void set_view(const ViewOpts &v) {
    {
      std::lock_guard<std::mutex> l(_m);
      _view = v;
      _dirty = true;
    }
    _cv.notify_one();
  }

  // The menu's backdrop: black, the picture moved into the preview. Eased in and out (the picture flies there and
  // back) unless `now`.
  void set_backdrop(bool on, bool now) {
    {
      std::lock_guard<std::mutex> l(_m);
      _inset_to = on ? 1.f : 0.f;
      if (now) _inset_snap = true;
      _dirty = true;
    }
    _cv.notify_one();
  }

  // What the card shown where there is no picture says: the chosen channel ("R8"), or "" for NO SIGNAL.
  void set_card_label(const std::string &label) {
    {
      std::lock_guard<std::mutex> l(_m);
      _label = label;
      _dirty = true;
    }
    _cv.notify_one();
  }

  // Where the menu's preview is on the window, in pixels, and its corner radius.
  void set_preview(const Preview &p) {
    {
      std::lock_guard<std::mutex> l(_m);
      _preview = p;
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
  static double now_s() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

  // Lines [y0, y0 + rows) of a w x h picture into the picture in hand. top_t / bottom_t: when the top / bottom line
  // of the picture was taken, if this brings it (0: not, or not known); newest_t: when the newest line in it was.
  // strip: from post_rows (the present waits for the compositor's next composition).
  void put(const uint8_t *src, int w, int h, int y0, int rows, double top_t, double bottom_t, double newest_t,
           bool strip = false) {
    if (w <= 0 || h <= 0 || y0 < 0 || rows <= 0 || y0 + rows > h) return;
    const double now = now_s();
    {
      std::lock_guard<std::mutex> l(_m);
      if (Clock::now() < _quiet_until) return;
      if (newest_t > 0 && _ages.size() < 4096) _ages.push_back(static_cast<float>((now - newest_t) * 1e3));
      _pending_strips = strip && (!_have || _pending_strips);  // a whole frame among them goes up at once
      if (!_have || _posted_t <= 0) _posted_t = now;
      _hold_until = {};
      const size_t rowb = static_cast<size_t>(w) * 4;
      if (w != _w || h != _h || _pending.size() != rowb * h) {  // a new shape: black until its lines come
        _w = w;
        _h = h;
        _pending.assign(rowb * h, 0);
        _dirty_lo = 0;
        _dirty_hi = h;
      }
      std::memcpy(_pending.data() + y0 * rowb, src, rows * rowb);
      _dirty_lo = std::min(_dirty_lo, y0);
      _dirty_hi = std::max(_dirty_hi, y0 + rows);
      if (top_t > 0) _top_t = top_t;
      if (bottom_t > 0) _bottom_t = bottom_t;
      _have = true;
    }
    _cv.notify_one();
  }

  // EGL_ANDROID_get_frame_timestamps on the surface just made: the display then says when each present went up.
  void timestamps_on() {
    _ts = false;
    _lat.forget();
    const char *ext = eglQueryString(_dpy, EGL_EXTENSIONS);
    if (!ext || !std::strstr(ext, "EGL_ANDROID_get_frame_timestamps")) {
      KP_LOGI("screen: the display gives no frame times (no EGL_ANDROID_get_frame_timestamps)");
      return;
    }
    _next_id = reinterpret_cast<PFNEGLGETNEXTFRAMEIDANDROIDPROC>(eglGetProcAddress("eglGetNextFrameIdANDROID"));
    _frame_ts = reinterpret_cast<PFNEGLGETFRAMETIMESTAMPSANDROIDPROC>(eglGetProcAddress("eglGetFrameTimestampsANDROID"));
    _ts_supported = reinterpret_cast<PFNEGLGETFRAMETIMESTAMPSUPPORTEDANDROIDPROC>(
        eglGetProcAddress("eglGetFrameTimestampSupportedANDROID"));
    _comp_timing = reinterpret_cast<PFNEGLGETCOMPOSITORTIMINGANDROIDPROC>(eglGetProcAddress("eglGetCompositorTimingANDROID"));
    if (!_next_id || !_frame_ts || !_ts_supported || !eglSurfaceAttrib(_dpy, _surf, EGL_TIMESTAMPS_ANDROID, EGL_TRUE) ||
        !_ts_supported(_dpy, _surf, EGL_DISPLAY_PRESENT_TIME_ANDROID)) {
      KP_LOGI("screen: the display gives no present times (0x%x)", eglGetError());
      return;
    }
    _ts = true;
    KP_LOGI("screen: frame times on (EGL_ANDROID_get_frame_timestamps)");
  }

  // The compositor's next composition after t (steady s), the deadline for a frame to be in it, and its own delay
  // from composition to the display (EGL_ANDROID_get_frame_timestamps' compositor timing, which it keeps up to date
  // with every present). False: not known.
  bool composition_after(double t, double &deadline, double &to_display) {
    if (!_comp_timing || _surf == EGL_NO_SURFACE) return false;
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

  // When the compositor expects to show a present handed over at t: its next composition, plus its delay to the
  // display. 0: not known.
  double expected_shown(double t) {
    double d, lat;
    return composition_after(t, d, lat) ? d + lat : 0;
  }

  // The display's refresh rate: the compositor's own interval, except with front-buffer drawing, where that is out
  // of date (set_display_hz) and the app's word is taken; failing both, the last known (0: none yet).
  float refresh_hz() {
    if (!_front && _comp_timing && _surf != EGL_NO_SURFACE) {
      const EGLint what = EGL_COMPOSITE_INTERVAL_ANDROID;
      EGLnsecsANDROID v = 0;
      if (_comp_timing(_dpy, _surf, 1, &what, &v) && v > 0) return static_cast<float>(1e9 / v);
    }
    const float app = _display_hz;
    return app > 0 ? app : _refresh_hz;
  }

  // When present id went up on the display (LatencyMeter::poll)
  int64_t display_time(uint64_t id) {
    const EGLint what = EGL_DISPLAY_PRESENT_TIME_ANDROID;
    EGLnsecsANDROID v = 0;
    if (!_frame_ts(_dpy, _surf, id, 1, &what, &v)) return LatencyMeter::kUnknown;
    if (v == EGL_TIMESTAMP_PENDING_ANDROID) return LatencyMeter::kPending;
    if (v == EGL_TIMESTAMP_INVALID_ANDROID) return LatencyMeter::kDropped;
    return v;
  }

  // Once a second: the window's numbers for the app, every kLogEvery of them into the log too.
  void latency_window(double now) {
    if (now - _lat_t0 < 1.0) return;
    _lat_t0 = now;
    LatencyStats s = _lat.take(now);
    _refresh_hz = refresh_hz();
    s.refresh_hz = _refresh_hz;
    {
      std::lock_guard<std::mutex> l(_m);
      _lat_shown = s;
      _front_shown = _front;
    }
    if (s.presents == 0 || ++_lat_logged % kLogEvery) return;
    KP_LOGI("screen%s: %d presents in %.1f s (%d shown, %d replaced, %d not known), display %.0f Hz; picture taken -> "
            "handed over: top %.1f ms, bottom %.1f ms; -> shown: top %.1f ms (95%% %.1f), bottom %.1f ms; -> expected "
            "shown: top %.1f ms, bottom %.1f ms; to the screen %.1f ms, render %.1f ms (swap %.1f)",
            _front ? " (front buffer: handed over = drawn on screen)" : "", s.presents, s.seconds, s.shown, s.dropped,
            s.unknown, s.refresh_hz, s.queued_top, s.queued_bottom,
            s.shown_top, s.shown_top_p95, s.shown_bottom, s.expected_top, s.expected_bottom, s.to_screen, s.render, s.swap);
  }

  static constexpr int kLogEvery = 10;           // latency_window: one window in this many into the log
  static constexpr float kInsetSeconds = 0.28f;  // the picture's way into the preview and back
  static constexpr double kStaleSeconds = 1.2;   // no frame for this long: the picture is not live any more ...
  static constexpr float kFadeSeconds = 0.5f;    // ... and fades into NO SIGNAL over this, dimming on the way ...
  static constexpr float kStaleBright = 0.6f;    // ... to this; while held (hold()) it stays up, only dimmed

  // A fade's next step toward `to`: by `up` if it rises, `down` if it falls.
  static float approach(float v, float to, float up, float down) {
    return v < to ? std::min(to, v + up) : std::max(to, v - down);
  }

  // EGL sorts the configs that match by colour depth, deepest first, so the first one is 10-bit wherever the
  // driver has such a config. The picture is 8-bit, and its buffer format then follows the phone, with a 2-bit
  // alpha the compositor may not put on an overlay. Of the configs that are exactly 8 bits a colour, take the
  // one with the least besides: no multisampling first (it costs the GPU on every frame), then opaque, then no
  // depth or stencil buffer (memory only; the emulator's EGL has no config without them). Only a driver with no
  // 8-bit config at all gets its first choice.
  // want_mutable (front-buffer drawing): one that can be made single-buffered (EGL_MUTABLE_RENDER_BUFFER_BIT_KHR)
  // comes before one with a depth buffer, but after opaque and no multisampling.
  static EGLConfig pick_config(EGLDisplay dpy, bool want_mutable) {
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
                       (get(c, EGL_STENCIL_SIZE) != 0 ? 1 : 0) +
                       (want_mutable && !(get(c, EGL_SURFACE_TYPE) & EGL_MUTABLE_RENDER_BUFFER_BIT_KHR) ? 4 : 0);
      if (cost < best_cost) best = c, best_cost = cost;
    }
    KP_LOGI("screen: EGL config R%d G%d B%d A%d, depth %d, stencil %d, samples %d, native visual 0x%x%s%s",
            get(best, EGL_RED_SIZE), get(best, EGL_GREEN_SIZE), get(best, EGL_BLUE_SIZE), get(best, EGL_ALPHA_SIZE),
            get(best, EGL_DEPTH_SIZE), get(best, EGL_STENCIL_SIZE), get(best, EGL_SAMPLES),
            get(best, EGL_NATIVE_VISUAL_ID), best_cost == 1 << 30 ? " (no 8-bit one: the driver's first)" : "",
            get(best, EGL_SURFACE_TYPE) & EGL_MUTABLE_RENDER_BUFFER_BIT_KHR ? ", can be single-buffered" : "");
    return best;
  }

  bool egl_up() {
    if (_dpy != EGL_NO_DISPLAY) return _ctx != EGL_NO_CONTEXT;
    _dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (_dpy == EGL_NO_DISPLAY || !eglInitialize(_dpy, nullptr, nullptr)) {
      KP_LOGE("screen: no EGL display (0x%x)", eglGetError());
      return false;
    }
    const char *ext = eglQueryString(_dpy, EGL_EXTENSIONS);
    _cfg = pick_config(_dpy, _front_req && ext && std::strstr(ext, "EGL_KHR_mutable_render_buffer"));
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
    timestamps_on();
    _tex_stale = true;  // the picture in hand goes up whole on the new surface
    if (!_gl_ready) {
      _gl_ready = _gl.init();
      if (!_gl_ready) KP_LOGE("screen: renderer did not start");
      else KP_LOGI("screen: OpenGL ES renderer ready");
      _tex_stale = true;
    }
    front_on();
  }

  // Front-buffer drawing on the surface just made, if it is asked for (set_front_buffer): EGL_KHR_mutable_render_buffer
  // makes the window surface single-buffered from the next swap (Android: the window's buffer queue in shared-buffer
  // mode), and EGL_ANDROID_front_buffer_auto_refresh has the compositor take that buffer at every refresh, queued or
  // not. Also, once: whether the other way to it is open on this device (a SurfaceControl layer with a buffer
  // allocated for front-buffer use, AHARDWAREBUFFER_USAGE_FRONT_BUFFER, as AndroidX's GLFrontBufferedRenderer does).
  void front_on() {
    _front = false;
    const char *ext = eglQueryString(_dpy, EGL_EXTENSIONS);
    const bool mut = ext && std::strstr(ext, "EGL_KHR_mutable_render_buffer");
    const bool autoref = ext && std::strstr(ext, "EGL_ANDROID_front_buffer_auto_refresh");
    static bool told = false;
    if (!told) {
      told = true;
      AHardwareBuffer_Desc d{};
      d.width = 1920;
      d.height = 1080;
      d.layers = 1;
      d.format = AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM;
      d.usage = AHARDWAREBUFFER_USAGE_GPU_FRAMEBUFFER | AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE |
                AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY | AHARDWAREBUFFER_USAGE_FRONT_BUFFER;
      KP_LOGI("screen: front-buffer ways: EGL_KHR_mutable_render_buffer %s, EGL_ANDROID_front_buffer_auto_refresh %s, "
              "front-buffer hardware buffers %s", mut ? "yes" : "no", autoref ? "yes" : "no",
              AHardwareBuffer_isSupported(&d) ? "yes" : "no");
    }
    if (!_front_req) return;
    EGLint st = 0;
    eglGetConfigAttrib(_dpy, _cfg, EGL_SURFACE_TYPE, &st);
    if (!mut || !(st & EGL_MUTABLE_RENDER_BUFFER_BIT_KHR)) {
      KP_LOGW("screen: front-buffer drawing asked for, but %s: the picture goes up as without it",
              mut ? "the EGL config cannot be single-buffered" : "EGL has no EGL_KHR_mutable_render_buffer");
      return;
    }
    if (!eglSurfaceAttrib(_dpy, _surf, EGL_RENDER_BUFFER, EGL_SINGLE_BUFFER)) {
      KP_LOGW("screen: front-buffer drawing: the surface cannot be single-buffered (0x%x)", eglGetError());
      return;
    }
    const bool ar = autoref && eglSurfaceAttrib(_dpy, _surf, EGL_FRONT_BUFFER_AUTO_REFRESH_ANDROID, EGL_TRUE);
    EGLint sw = 0, sh = 0;
    eglQuerySurface(_dpy, _surf, EGL_WIDTH, &sw);
    eglQuerySurface(_dpy, _surf, EGL_HEIGHT, &sh);
    VideoRenderer::blank(0, sw, sh);
    eglSwapBuffers(_dpy, _surf);  // single-buffered from here on
    EGLint rb = 0;
    eglQueryContext(_dpy, _ctx, EGL_RENDER_BUFFER, &rb);
    _front = rb == EGL_SINGLE_BUFFER;
    _front_whole = true;
    if (_front) _ts = false;  // no queue of frames: nothing for the display to give times of
    KP_LOGI("screen: front-buffer drawing %s (compositor auto refresh %s)",
            _front ? "on" : "asked for, but the surface stayed double-buffered", ar ? "on" : "off");
  }

  // EGL and GL from scratch on the same window (the GPU was reset under us, or front-buffer drawing was switched:
  // it may need another config). context_alive: the GL objects can be freed (else they are forgotten).
  void restart_egl(bool context_alive) {
    if (_gl_ready) _gl.destroy(context_alive && _surf != EGL_NO_SURFACE);
    _gl_ready = false;
    ANativeWindow *keep = _win;
    if (keep) ANativeWindow_acquire(keep);
    drop_surface();
    if (_dpy != EGL_NO_DISPLAY) {
      if (_ctx != EGL_NO_CONTEXT) eglDestroyContext(_dpy, _ctx);
      eglTerminate(_dpy);
    }
    _ctx = EGL_NO_CONTEXT;
    _dpy = EGL_NO_DISPLAY;
    _front = false;
    if (keep) take_window(keep);
  }

  void loop() {
    pthread_setname_np(pthread_self(), "fpv screen");  // its CPU, apart from the others' (top -H)
    KP_LOGI("screen: render thread %d", static_cast<int>(gettid()));
    std::vector<uint8_t> last;  // the picture in hand, whole (rows go up as they change; all of it after a GL restart)
    int lw = 0, lh = 0;
    std::vector<float> ages;    // LatencyMeter::handed
    float inset = 0, gain = 0, bright = 1;  // gain: the picture against NO SIGNAL; bright: the picture's own
    auto t_prev = Clock::now(), t_frame = t_prev;
    bool live = false;  // a frame within kStaleSeconds
    bool held = false;  // a stale picture stays up, dimmed, instead of fading into NO SIGNAL (hold())
    // Where the fades are headed: a live picture all the way up; a stale one dimmed (never brightened) and, unless
    // it is held, gone into NO SIGNAL.
    const auto gain_to = [&] { return live ? 1.f : held ? gain : 0.f; };
    const auto bright_to = [&] { return live ? 1.f : std::min(bright, kStaleBright); };
    for (;;) {
      bool got = false, cleared = false, dirtied = false, animating;
      int lo = 0, hi = 0;              // the rows of `last` that changed (got)
      double top_t = 0, bottom_t = 0;  // when the picture's top / bottom line that came now was taken
      double posted_t = 0;             // when the first of it was handed to the screen
      ViewOpts view;
      Scene scene;
      float inset_to;
      {
        std::unique_lock<std::mutex> l(_m);
        const auto ready = [&] { return _stop || _win_next_set || _have || _clear || _dirty || _rebuild; };
        held = Clock::now() < _hold_until;
        animating = inset != _inset_to || gain != gain_to() || bright != bright_to();
        if (animating) _cv.wait_for(l, std::chrono::milliseconds(8), ready);
        else if (live) _cv.wait_until(l, t_frame + std::chrono::duration<double>(kStaleSeconds), ready);
        else if (held) _cv.wait_until(l, _hold_until, ready);  // then into NO SIGNAL, unless a frame came
        else _cv.wait(l, ready);
        // strips go up just before the compositor's next composition (set_present): more pile up meanwhile
        // (the first composition still margin away: one closer than that would most likely be missed anyway, and
        // the strips that come until the next go up with it)
        double deadline, to_display;
        const double margin = _margin_us.load(std::memory_order_relaxed) * 1e-6;
        if (_have && _pending_strips && _align && !_front && !_stop && !_win_next_set && !_rebuild &&
            composition_after(now_s() + margin, deadline, to_display)) {
          const double at = deadline - margin;
          if (now_s() < at)
            _cv.wait_until(l, Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(at))),
                           [&] { return _stop || _win_next_set || _clear; });
        }
        if (_stop) break;
        held = Clock::now() < _hold_until;
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
        if (_rebuild) {  // front-buffer drawing switched (set_front_buffer)
          _rebuild = false;
          l.unlock();
          restart_egl(true);
          l.lock();
        }
        if (_have) {
          const size_t rowb = static_cast<size_t>(_w) * 4;
          if (lw != _w || lh != _h || last.size() != _pending.size()) {
            last.assign(_pending.size(), 0);
            lw = _w;
            lh = _h;
            _dirty_lo = 0;
            _dirty_hi = _h;
          }
          lo = std::clamp(_dirty_lo, 0, _h);
          hi = std::clamp(_dirty_hi, lo, _h);
          std::memcpy(last.data() + lo * rowb, _pending.data() + lo * rowb, (hi - lo) * rowb);
          _dirty_lo = _h;
          _dirty_hi = 0;
          top_t = _top_t;
          bottom_t = _bottom_t;
          posted_t = _posted_t;
          _top_t = _bottom_t = _posted_t = 0;
          ages.swap(_ages);
          _ages.clear();
          _have = false;
          got = true;
        }
        cleared = _clear;
        _clear = false;
        dirtied = _dirty;
        _dirty = false;
        view = _view;
        scene.preview = _preview;
        scene.label = _label;
        inset_to = _inset_to;
        if (_inset_snap) inset = inset_to;
        _inset_snap = false;
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
      // ease the picture's way into the preview, and its fade, toward where they are headed: a live picture
      // comes up four times as fast as a stale one goes
      const float istep = dt / kInsetSeconds, gstep = dt / kFadeSeconds;
      inset = approach(inset, inset_to, istep, istep);
      gain = approach(gain, gain_to(), 4 * gstep, gstep);
      bright = approach(bright, bright_to(), 4 * gstep, gstep);
      if (_surf == EGL_NO_SURFACE || !_gl_ready) {
        if (got) _tex_stale = true;  // rows that changed meanwhile: all of it goes up when there is a surface
        continue;
      }
      if ((got || _tex_stale) && !last.empty()) {
        if (_tex_stale) lo = 0, hi = lh;
        _gl.upload(last.data(), lw, lh, lo, hi);
        _tex_stale = false;
      }
      if (cleared && !got) _gl.clear();
      EGLint sw = 0, sh = 0;
      eglQuerySurface(_dpy, _surf, EGL_WIDTH, &sw);
      eglQuerySurface(_dpy, _surf, EGL_HEIGHT, &sh);
      scene.inset = inset * inset * (3 - 2 * inset);  // smoothstep
      scene.gain = gain;
      scene.bright = bright;
      // Front-buffer drawing: with the picture steady (full size, all of it, no fade) and drawn whole on this
      // surface before, only the band of the screen that its changed rows cover is drawn again.
      const bool steady = inset == inset_to && gain == gain_to() && bright == bright_to() && inset <= 0 && gain >= 1 &&
                          bright >= 1 && !cleared && !dirtied && sw == _front_w && sh == _front_h;
      const bool band = _front && !_front_whole && steady && got && lh > 0 && hi > lo && (lo > 0 || hi < lh);
      _gl.draw(0, sw, sh, view, scene, band ? static_cast<float>(lo) / lh : 0.f, band ? static_cast<float>(hi) / lh : 1.f);
      if (!band) _front_whole = false, _front_w = sw, _front_h = sh;
      EGLuint64KHR id = 0;
      double t_drawn = 0;
      if (_front) {
        _gl.wait_drawn(20);  // then it is in the buffer the display shows
        t_drawn = now_s();
      } else if (_ts && got && !_next_id(_dpy, _surf, &id)) {
        id = 0;
      }
      const double t_pre = now_s();
      const bool swapped = eglSwapBuffers(_dpy, _surf);  // front buffer: tells the compositor it changed
      const double t_swap = now_s();
      const float swap_ms = static_cast<float>((t_swap - t_pre) * 1e3);
      if (swapped && got && _front) {  // up as the display next scans there: half a refresh later on average
        const float hz = refresh_hz();
        _lat.presented(t_drawn, top_t, bottom_t, 0, t_drawn + 0.5 / (hz > 0 ? hz : 60.f), posted_t, swap_ms);
      } else if (swapped && got)
        _lat.presented(t_swap, top_t, bottom_t, id, expected_shown(t_swap), posted_t, swap_ms);
      if (got) _lat.handed(ages);
      if (_ts) _lat.poll(t_swap, [this](uint64_t i) { return display_time(i); });
      latency_window(t_swap);
      if (!swapped) {
        const EGLint e = eglGetError();
        KP_LOGW("screen: swap failed (0x%x)", e);
        if (e == EGL_CONTEXT_LOST) restart_egl(false);  // the GPU was reset under us: everything GL is gone
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
  std::vector<uint8_t> _pending;  // the newest picture, whole: frames and strips go into it
  int _w = 0, _h = 0;
  int _dirty_lo = 0, _dirty_hi = 0;  // its rows that changed since the render thread last took them
  double _top_t = 0, _bottom_t = 0;  // when the top / bottom line that came since was taken (0: none came)
  double _posted_t = 0;              // when the first of what came since was handed over
  std::vector<float> _ages;          // the newest line in each strip or frame since: taken -> handed over (ms)
  LatencyStats _lat_shown;           // the last window's (latency_json)
  bool _pending_strips = false;      // all that came since the last present were strips
  std::atomic<bool> _align{true};    // set_present
  std::atomic<int> _margin_us{2500};
  bool _have = false, _clear = false, _dirty = false, _stop = false;
  std::atomic<bool> _front_req{false};        // set_front_buffer (read by the render thread outside _m)
  bool _rebuild = false;                      // ... EGL to be started again
  bool _front_shown = false;                  // front-buffer drawing as of the last window (latency_json)
  std::atomic<float> _display_hz{0};          // set_display_hz (0: not said)
  std::chrono::steady_clock::time_point _quiet_until{};
  std::chrono::steady_clock::time_point _hold_until{};
  ViewOpts _view;
  float _inset_to = 0;
  bool _inset_snap = false;
  Preview _preview;
  std::string _label;
  // render thread only
  EGLDisplay _dpy = EGL_NO_DISPLAY;
  EGLConfig _cfg = nullptr;
  EGLContext _ctx = EGL_NO_CONTEXT;
  EGLSurface _surf = EGL_NO_SURFACE;
  ANativeWindow *_win = nullptr;
  VideoRenderer _gl;
  bool _gl_ready = false, _tex_stale = false;
  bool _front = false;        // front-buffer drawing is on (front_on)
  bool _front_whole = true;   // ... and the next draw must be of the whole screen
  EGLint _front_w = 0, _front_h = 0;  // the surface's size at the last whole draw
  // the picture's delay (latency.h); the display's present times where it gives them (timestamps_on)
  LatencyMeter _lat;
  double _lat_t0 = 0;
  uint64_t _lat_logged = 0;
  float _refresh_hz = 0;
  bool _ts = false;
  PFNEGLGETNEXTFRAMEIDANDROIDPROC _next_id = nullptr;
  PFNEGLGETFRAMETIMESTAMPSANDROIDPROC _frame_ts = nullptr;
  PFNEGLGETFRAMETIMESTAMPSUPPORTEDANDROIDPROC _ts_supported = nullptr;
  PFNEGLGETCOMPOSITORTIMINGANDROIDPROC _comp_timing = nullptr;
  std::thread _th;
};

}  // namespace kp
