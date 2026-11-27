// fpvDROID: the app's screen on Android (the Kotlin app's Surface; fpvdroid_jni.cpp hands it frames and settings).
// Frames go straight to the video Surface: a render thread owns an EGL context on it and draws each frame as
// it arrives (latest frame wins; the decode thread never waits for the display, only for its lines to be copied).
// With Vsync off (PictureOpts::strips) the picture comes in strips instead, and only the rows that changed go to
// the GPU. Where the phone can, each strip is drawn as it comes, straight into the buffer the display shows
// (front-buffer drawing, front_buffer.h). Where it cannot, the strips go up once a refresh, just before the
// compositor takes its next frame, with the strips that have come by then.
//
// How old the picture is when it is shown is measured as it goes (latency.h), with the display's own timing where
// it gives any (display_timing.h).
#pragma once
#include <EGL/egl.h>
#include <android/native_window.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "display_timing.h"
#include "front_buffer.h"
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

  // Decode thread: copy and go. The frame's new lines into the picture in hand: all of them, or with low latency a
  // strip of it (Frame::y0, y1), the rest as it was. Up with the next present.
  void post(const Frame &f) {
    if (f.width <= 0 || f.height <= 0 || f.y0 < 0 || f.y1 <= f.y0 || f.y1 > f.height) return;
    const bool strip = f.y0 > 0 || f.y1 < f.height;
    const double now = now_s();
    {
      std::lock_guard<std::mutex> l(_m);
      if (Clock::now() < _quiet_until) return;
      if (f.times.t_last > 0 && _ages.size() < 4096) _ages.push_back(static_cast<float>((now - f.times.t_last) * 1e3));
      _pending_strips = strip && (!_have || _pending_strips);  // a whole picture among them goes up at once
      if (!_have || _posted_t <= 0) _posted_t = now;
      _hold_until = {};
      const size_t rowb = static_cast<size_t>(f.width) * 4;
      if (f.width != _w || f.height != _h || _pending.size() != rowb * f.height) {
        // a new shape: black until its lines come
        _w = f.width;
        _h = f.height;
        _pending.assign(rowb * f.height, 0);
        _pending_behind = false;
        _dirty_lo = 0;
        _dirty_hi = f.height;
      }
      // the new lines; where the render thread took the buffer that had the rest (take_picture), all of them
      const int y0 = _pending_behind ? 0 : f.y0, y1 = _pending_behind ? f.height : f.y1;
      std::memcpy(_pending.data() + y0 * rowb, f.data + y0 * rowb, (y1 - y0) * rowb);
      _pending_behind = false;
      _dirty_lo = std::min(_dirty_lo, f.y0);
      _dirty_hi = std::max(_dirty_hi, f.y1);
      if (f.y0 == 0 && f.times.t_first > 0) _top_t = f.times.t_first;
      if (f.finished() && f.times.t_last > 0) _bottom_t = f.times.t_last;
      _have = true;
    }
    _cv.notify_one();
  }

  // Debug (KP_PRESENT_ALIGN), for strips that go through the compositor (no front-buffer drawing): they go up once
  // a refresh, just before the compositor's next composition (on, the default), or each as it comes (off).
  // Measured on a Pixel 8 Pro at 120 Hz: a present for every strip (~350 a second) made the compositor show each
  // frame a refresh later than one a refresh did, and the display's history of frame times was too short to say
  // when strips went up.
  void set_present_align(bool on) {
    _align = on;
    _cv.notify_one();
  }
  // Debug (KP_PRESENT_MARGIN_US): ... and how long before that composition.
  void set_present_margin_us(int us) {
    _margin_us = std::clamp(us, 0, 20000);
    _cv.notify_one();
  }

  // The picture comes in strips (PictureOpts::strips, Vsync off): where the phone can, they are drawn straight into
  // the buffer the display shows (front_buffer.h), from a fresh start of EGL on the window.
  void set_strips(bool on) {
    std::lock_guard<std::mutex> l(_m);
    _front_wanted = on;
    front_changed();
  }
  // Debug (KP_FRONT_BUFFER=0): never that. Strips then go up once a refresh, as on a phone that cannot.
  void allow_front_buffer(bool on) {
    std::lock_guard<std::mutex> l(_m);
    _front_allowed = on;
    front_changed();
  }

  // The display's refresh rate as the app sees it (its Display's mode), each time that changes. The readout takes
  // it where the compositor's own timing is out of date: with front-buffer drawing no frames are queued, and
  // nothing brings that timing up to date after a mode change (on a Pixel 8 Pro it said 120 Hz after the panel
  // went to 60).
  void set_display_hz(float hz) { _display_hz = hz > 0 ? hz : 0.f; }

  // The picture's delay over the last second or so (latency.h), as JSON for the app.
  std::string latency_json() {
    LatencyStats s;
    {
      std::lock_guard<std::mutex> l(_m);
      s = _lat_shown;
    }
    return kp::latency_json(s);
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

  static constexpr int kLogEvery = 10;           // latency_window: one window in this many into the log
  static constexpr float kInsetSeconds = 0.28f;  // the picture's way into the preview and back
  static constexpr double kStaleSeconds = 1.2;   // no frame for this long: the picture is not live any more ...
  static constexpr float kFadeSeconds = 0.5f;    // ... and fades into NO SIGNAL over this, dimming on the way ...
  static constexpr float kStaleBright = 0.6f;    // ... to this; while held (hold()) it stays up, only dimmed

  // Under _m: front-buffer drawing is asked for ...
  bool front_asked() const { return _front_wanted && _front_allowed; }
  // ... and where that is not what EGL was last started for (egl_up), it is started again. Not on a phone whose EGL
  // has no way to it: nothing would change.
  void front_changed() {
    if (!_front_possible || front_asked() == _front_built) return;
    _rebuild = true;
    _cv.notify_one();
  }

  // A fade's next step toward `to`: by `up` if it rises, `down` if it falls.
  static float approach(float v, float to, float up, float down) {
    return v < to ? std::min(to, v + up) : std::max(to, v - down);
  }

  // What the render thread takes of the picture each time round (take_picture).
  struct Taken {
    bool got = false;                // lines of the picture came: rows [lo, hi) of _last are new
    int lo = 0, hi = 0;
    double top_t = 0, bottom_t = 0;  // when the picture's top / bottom line that came now was taken (0: none came)
    double posted_t = 0;             // when the first of it was handed to the screen
  };

  // Under _m: the picture's lines that came since the last time, into the render thread's own copy of it (_last).
  Taken take_picture() {
    Taken t;
    if (!_have) return t;
    const size_t rowb = static_cast<size_t>(_w) * 4;
    if (_lw != _w || _lh != _h || _last.size() != _pending.size()) {
      _last.assign(_pending.size(), 0);
      _lw = _w;
      _lh = _h;
      _dirty_lo = 0;
      _dirty_hi = _h;
    }
    t.lo = std::clamp(_dirty_lo, 0, _h);
    t.hi = std::clamp(_dirty_hi, t.lo, _h);
    if (t.lo == 0 && t.hi == _h) {
      // all of it: the two buffers change places, and the next frame fills _pending whole (post)
      _last.swap(_pending);
      _pending_behind = true;
    } else {
      std::memcpy(_last.data() + t.lo * rowb, _pending.data() + t.lo * rowb, (t.hi - t.lo) * rowb);
    }
    _dirty_lo = _h;
    _dirty_hi = 0;
    t.top_t = _top_t;
    t.bottom_t = _bottom_t;
    t.posted_t = _posted_t;
    _top_t = _bottom_t = _posted_t = 0;
    _taken_ages.swap(_ages);
    _ages.clear();
    _have = false;
    t.got = true;
    return t;
  }

  // Under _m, on the render thread. Strips that are waiting go up together, just before the compositor's next
  // composition (set_present_align): when that is (steady clock, s). 0: what is in hand goes up now. A composition
  // nearer than the margin would most likely be missed anyway: the one after it is taken, and the strips that come
  // until then go up with it.
  double present_due() const {
    if (!_have || !_pending_strips || !_align || _front.on() || _stop || _win_next_set || _rebuild) return 0;
    const double margin = _margin_us * 1e-6;
    double deadline, to_display;
    return _timing.composition_after(now_s() + margin, deadline, to_display) ? deadline - margin : 0;
  }

  // The display's refresh rate: the compositor's own, or the app's word (set_display_hz) where the compositor says
  // nothing or, with front-buffer drawing, is out of date; failing both, the last known (0: none yet).
  float refresh_hz() {
    float hz = _front.on() ? 0.f : _timing.refresh_hz();
    if (hz <= 0) hz = _display_hz;
    if (hz > 0) _refresh_hz = hz;
    return _refresh_hz;
  }

  // Once a second: the window's numbers for the app, every kLogEvery of them into the log too.
  void latency_window(double now) {
    if (now - _lat_t0 < 1.0) return;
    _lat_t0 = now;
    const LatencyStats s = _lat.take(now, refresh_hz(), _front.on());
    {
      std::lock_guard<std::mutex> l(_m);
      _lat_shown = s;
    }
    if (s.presents > 0 && ++_lat_logged % kLogEvery == 0) KP_LOGI("%s", latency_log(s).c_str());
  }

  // EGL sorts the configs that match by colour depth, deepest first, so the first one is 10-bit wherever the
  // driver has such a config. The picture is 8-bit, and its buffer format then follows the phone, with a 2-bit
  // alpha the compositor may not put on an overlay. Of the configs that are exactly 8 bits a colour, take the
  // one with the least besides: no multisampling first (it costs the GPU on every frame), then opaque, then no
  // depth or stencil buffer (memory only; the emulator's EGL has no config without them). Only a driver with no
  // 8-bit config at all gets its first choice.
  // want_front (front-buffer drawing): one that can be made single-buffered comes before one with a depth buffer,
  // but after opaque and no multisampling.
  static EGLConfig pick_config(EGLDisplay dpy, bool want_front) {
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
                       (want_front && !FrontBuffer::config_can(dpy, c) ? 4 : 0);
      if (cost < best_cost) best = c, best_cost = cost;
    }
    KP_LOGI("screen: EGL config R%d G%d B%d A%d, depth %d, stencil %d, samples %d, native visual 0x%x%s%s",
            get(best, EGL_RED_SIZE), get(best, EGL_GREEN_SIZE), get(best, EGL_BLUE_SIZE), get(best, EGL_ALPHA_SIZE),
            get(best, EGL_DEPTH_SIZE), get(best, EGL_STENCIL_SIZE), get(best, EGL_SAMPLES),
            get(best, EGL_NATIVE_VISUAL_ID), best_cost == 1 << 30 ? " (no 8-bit one: the driver's first)" : "",
            FrontBuffer::config_can(dpy, best) ? ", can be single-buffered" : "");
    return best;
  }

  bool egl_up() {
    if (_dpy != EGL_NO_DISPLAY) return _ctx != EGL_NO_CONTEXT;
    _dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (_dpy == EGL_NO_DISPLAY || !eglInitialize(_dpy, nullptr, nullptr)) {
      KP_LOGE("screen: no EGL display (0x%x)", eglGetError());
      return false;
    }
    {
      std::lock_guard<std::mutex> l(_m);  // what this start of EGL is for: front_changed asks for another
      _front_possible = FrontBuffer::available(_dpy);
      _front_built = front_asked();
    }
    _cfg = pick_config(_dpy, _front_built && _front_possible);
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
    _timing.detach();
    _front.stop();
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
    _timing.attach(_dpy, _surf);
    _lat.forget();
    if (!_gl_ready) {
      _gl_ready = _gl.init();
      if (!_gl_ready) KP_LOGE("screen: renderer did not start");
      else KP_LOGI("screen: OpenGL ES renderer ready");
    }
    // nothing of the picture is on the new surface: all of it goes up, and is drawn whole
    _tex_stale = true;
    _gl.forget_drawn();
    // front-buffer drawing queues no frames: nothing for the display to give times of
    if (_front_built && _front.start(_dpy, _surf, _ctx, _cfg)) _timing.stop_frame_times();
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
    if (keep) take_window(keep);
  }

  // The scene onto the sw x sh surface and handed over, and the picture's delay noted. With front-buffer drawing,
  // only the part of the screen that shows the picture's new rows, where nothing else has changed (not `all`).
  void present(const Taken &in, const ViewOpts &view, const Scene &scene, EGLint sw, EGLint sh, bool all) {
    const bool front = _front.on();
    const bool band = front && in.got && !all && _gl.draw_band(0, sw, sh, view, scene, in.lo, in.hi);
    if (!band) _gl.draw(0, sw, sh, view, scene);
    LatencyMeter::Present p{.top_t = in.top_t, .bottom_t = in.bottom_t, .posted = in.posted_t};
    if (front) {
      FrontBuffer::wait_drawn(20);  // then it is in the buffer the display shows
      p.t = now_s();
    } else if (in.got) {
      p.id = _timing.next_frame_id();
    }
    const double t_pre = now_s();
    const bool swapped = eglSwapBuffers(_dpy, _surf);  // front buffer: tells the compositor it changed
    const double t_swap = now_s();
    p.swap_ms = static_cast<float>((t_swap - t_pre) * 1e3);
    if (front) {  // up as the display next scans there: half a refresh later on average
      const float hz = refresh_hz();
      p.expected = p.t + 0.5 / (hz > 0 ? hz : 60.f);
    } else {
      p.t = t_swap;
      p.expected = _timing.expected_shown(t_swap);
    }
    if (swapped && in.got) _lat.presented(p);
    if (in.got) _lat.handed(_taken_ages);
    if (_timing.frame_times()) _lat.poll(t_swap, [this](uint64_t id) { return _timing.shown_at(id); });
    latency_window(t_swap);
    if (!swapped) {
      const EGLint e = eglGetError();
      KP_LOGW("screen: swap failed (0x%x)", e);
      if (e == EGL_CONTEXT_LOST) restart_egl(false);  // the GPU was reset under us: everything GL is gone
    }
  }

  void loop() {
    pthread_setname_np(pthread_self(), "fpv screen");  // its CPU, apart from the others' (top -H)
    KP_LOGI("screen: render thread %d", static_cast<int>(gettid()));
    float inset = 0, gain = 0, bright = 1;  // gain: the picture against NO SIGNAL; bright: the picture's own
    auto t_prev = Clock::now(), t_frame = t_prev;
    bool live = false;  // a frame within kStaleSeconds
    bool held = false;  // a stale picture stays up, dimmed, instead of fading into NO SIGNAL (hold())
    // Where the fades are headed: a live picture all the way up; a stale one dimmed (never brightened) and, unless
    // it is held, gone into NO SIGNAL.
    const auto gain_to = [&] { return live ? 1.f : held ? gain : 0.f; };
    const auto bright_to = [&] { return live ? 1.f : std::min(bright, kStaleBright); };
    for (;;) {
      Taken in;
      bool cleared = false, dirtied = false, animating;
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
        // strips wait for their present: more pile up meanwhile
        if (const double at = present_due(); at > now_s())
          _cv.wait_until(l, Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(at))),
                         [&] { return _stop || _win_next_set || _clear; });
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
        if (_rebuild) {  // front-buffer drawing switched (set_strips, allow_front_buffer)
          _rebuild = false;
          if (front_asked() != _front_built) {  // and not switched back since
            l.unlock();
            restart_egl(true);
            l.lock();
          }
        }
        in = take_picture();
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
      if (cleared && !in.got) {
        live = false;
        gain = 0;
        _last.clear();
      }
      if (in.got) {
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
        if (in.got) _tex_stale = true;  // rows that changed meanwhile: all of it goes up when there is a surface
        continue;
      }
      if ((in.got || _tex_stale) && !_last.empty()) {
        if (_tex_stale) in.lo = 0, in.hi = _lh;
        _gl.upload(_last.data(), _lw, _lh, in.lo, in.hi);
        _tex_stale = false;
      }
      if (cleared && !in.got) _gl.clear();
      EGLint sw = 0, sh = 0;
      eglQuerySurface(_dpy, _surf, EGL_WIDTH, &sw);
      eglQuerySurface(_dpy, _surf, EGL_HEIGHT, &sh);
      scene.inset = inset * inset * (3 - 2 * inset);  // smoothstep
      scene.gain = gain;
      scene.bright = bright;
      present(in, view, scene, sw, sh, cleared || dirtied);
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
  std::vector<uint8_t> _pending;     // the newest picture, whole: frames and strips go into it
  int _w = 0, _h = 0;
  int _dirty_lo = 0, _dirty_hi = 0;  // its rows that changed since the render thread last took them
  bool _pending_behind = false;      // ... which took all of it, buffer and all: _pending is an older picture
  double _top_t = 0, _bottom_t = 0;  // when the top / bottom line that came since was taken (0: none came)
  double _posted_t = 0;              // when the first of what came since was handed over
  std::vector<float> _ages;          // the newest line in each frame since: taken -> handed over (ms)
  LatencyStats _lat_shown;           // the last window's (latency_json)
  bool _pending_strips = false;      // all that came since the last present were strips
  std::atomic<bool> _align{true};    // set_present_align
  std::atomic<int> _margin_us{2500};  // set_present_margin_us
  bool _have = false, _clear = false, _dirty = false, _stop = false;
  bool _front_wanted = false, _front_allowed = true;  // set_strips, allow_front_buffer
  // Front-buffer drawing as EGL was last started for it, and whether this EGL has the way to it (true until EGL
  // says no). Written by the render thread alone (egl_up), which reads them without the lock.
  bool _front_built = false, _front_possible = true;
  bool _rebuild = false;              // EGL to be started again (front_changed)
  std::atomic<float> _display_hz{0};  // set_display_hz (0: not said)
  Clock::time_point _quiet_until{};
  Clock::time_point _hold_until{};
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
  std::vector<uint8_t> _last;  // the picture in hand, whole (rows go up as they change; all of it after a GL restart)
  int _lw = 0, _lh = 0;
  FrontBuffer _front;          // front-buffer drawing, where it is on
  DisplayTiming _timing;       // the display's own timing, where it gives any
  // the picture's delay (latency.h)
  LatencyMeter _lat;
  std::vector<float> _taken_ages;  // LatencyMeter::handed: what take_picture took of _ages
  double _lat_t0 = 0;
  uint64_t _lat_logged = 0;
  float _refresh_hz = 0;  // the display's, as last known (refresh_hz)
  std::thread _th;
};

}  // namespace kp
