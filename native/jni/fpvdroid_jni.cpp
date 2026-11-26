// fpvDROID JNI: the Kotlin app's view of the radio and the screen (com.fpvdroid.app.Native).
#include <android/native_window_jni.h>
#include <jni.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "log.h"
#include "radio.h"
#include "screen.h"
#include "test_card.h"
#include "usbmode.h"
#include "video.h"
#include "video_renderer.h"

using namespace kp;

namespace {

// ---- log file ----
std::mutex g_log_m;
FILE *g_log = nullptr;
void file_sink(LogLevel l, const char *line) {
  std::lock_guard<std::mutex> lk(g_log_m);
  if (!g_log) return;
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  tm t;
  localtime_r(&ts.tv_sec, &t);
  static const char tag[] = {'D', 'I', 'W', 'E'};
  std::fprintf(g_log, "%02d:%02d:%02d.%03ld %c %s\n", t.tm_hour, t.tm_min, t.tm_sec, ts.tv_nsec / 1000000,
               tag[static_cast<int>(l)], line);
  std::fflush(g_log);
}

std::unique_ptr<Screen> g_screen;
std::unique_ptr<Radio> g_radio;
// Keeps an open and a close from overlapping. Only the worker that opens and closes takes it: the mode calls (scan,
// watch, idle, streamTest) come from the main thread, which must never wait out a radio bring-up, and they need
// no more than the radio's own lock (each only sets the mode, the radio ignores them while it is not open, and
// an open or close sets the mode itself when it is done).
std::mutex g_api;

// ---- test picture ----
// The screen side of the app without an adapter (the "testcard" extra): the test card as if it were decoded
// video, 30 times a second, with its bar moving.
std::mutex g_card_m;
std::thread g_card;
std::atomic<bool> g_card_on{false};

void card_stop() {
  std::lock_guard<std::mutex> l(g_card_m);
  g_card_on = false;
  if (g_card.joinable()) g_card.join();
}

void card_start() {
  card_stop();
  std::lock_guard<std::mutex> l(g_card_m);
  g_card_on = true;
  g_card = std::thread([] {
    std::vector<uint8_t> px;
    for (int tick = 1; g_card_on; tick++) {
      test_card(px, 720, 480, tick);
      if (g_screen) g_screen->post(Frame{px.data(), 720, 480, &kNtsc, 0, 480});
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  });
}

// ---- debug switches for wifi-sdr / devourer (the "env" extra: WSDR_URB_DEPTH and the like) ----
// They read theirs from the environment, on the radio's threads, and setenv is not safe beside a getenv on another
// thread. So the ones asked for wait here, and go into the environment in Native_open with the radio closed.
std::mutex g_env_m;
std::vector<std::pair<std::string, std::string>> g_env_wanted;  // name, value ("" removes it)

void apply_env() {
  std::lock_guard<std::mutex> l(g_env_m);
  for (const auto &[k, v] : g_env_wanted) {
    if (v.empty()) unsetenv(k.c_str());
    else setenv(k.c_str(), v.c_str(), 1);
  }
  g_env_wanted.clear();
}

std::string jstring_of(JNIEnv *env, jstring s) {
  const char *p = env->GetStringUTFChars(s, nullptr);
  std::string out = p ? p : "";
  if (p) env->ReleaseStringUTFChars(s, p);
  return out;
}

// wifi-sdr reports what its stream does (a start that had to be repeated, a bank dropped, where the chip's CPU got
// to) on stderr, which Android sends nowhere. Into the log with it, a line at a time.
void forward_stderr() {
  int p[2];
  if (pipe(p) != 0) return;
  if (dup2(p[1], STDERR_FILENO) < 0) {
    close(p[0]);
    close(p[1]);
    return;
  }
  close(p[1]);
  setvbuf(stderr, nullptr, _IOLBF, 0);
  std::thread([fd = p[0]] {
    std::string line;
    char buf[512];
    for (;;) {
      const ssize_t n = read(fd, buf, sizeof buf);
      if (n <= 0) return;
      for (ssize_t i = 0; i < n; i++) {
        if (buf[i] != '\n') {
          line += buf[i];
          continue;
        }
        if (!line.empty()) KP_LOGW("wsdr: %s", line.c_str());
        line.clear();
      }
    }
  }).detach();
}
}  // namespace

// Native is a Kotlin object: these are its instance methods, and the second argument is that one instance
// (unused), whatever its type is spelled as.
extern "C" {

// Every new activity calls this on its main thread, possibly while an earlier one's radio is still being opened
// (seconds, or for ever if the adapter hangs): so not under g_api, and the screen and radio are made only once.
JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_init(JNIEnv *env, jclass, jstring log_path) {
  const std::string p = jstring_of(env, log_path);
  {
    std::lock_guard<std::mutex> lk(g_log_m);
    if (!g_log) g_log = std::fopen(p.c_str(), "a");
  }
  set_log_sink(file_sink);
  static std::once_flag made;
  std::call_once(made, [] {
    forward_stderr();
    g_screen = std::make_unique<Screen>();
    g_radio = std::make_unique<Radio>([](const Frame &f) { g_screen->post(f); });
  });
  KP_LOGI("fpvDROID native ready");
}

JNIEXPORT jstring JNICALL Java_com_fpvdroid_app_Native_usb3Switch(JNIEnv *env, jclass, jint fd) {
  std::string detail;
  const auto r = wsdr::request_usb3_fd(fd, &detail);
  std::string out = wsdr::usb3_switch_name(r);
  if (!detail.empty()) out += ": " + detail;
  KP_LOGI("usb3 switch: %s", out.c_str());
  return env->NewStringUTF(out.c_str());
}

JNIEXPORT jstring JNICALL Java_com_fpvdroid_app_Native_open(JNIEnv *env, jclass, jint fd) {
  std::lock_guard<std::mutex> l(g_api);
  try {
    g_radio->close();  // open() would; before the environment changes, so that none of its threads is reading it
    apply_env();
    g_radio->open(fd);
    return env->NewStringUTF("");
  } catch (const std::exception &e) {
    KP_LOGE("open failed: %s", e.what());
    return env->NewStringUTF(e.what()[0] ? e.what() : "open failed");
  }
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_close(JNIEnv *, jclass) {
  std::lock_guard<std::mutex> l(g_api);
  if (g_radio) g_radio->close();
  if (g_screen) g_screen->clear();
}

JNIEXPORT jboolean JNICALL Java_com_fpvdroid_app_Native_superspeed(JNIEnv *, jclass) {
  return g_radio && g_radio->superspeed();
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_scan(JNIEnv *, jclass, jdouble prefer_hz) {
  if (g_radio) g_radio->scan(prefer_hz);
  if (g_screen) g_screen->clear();
}

// fresh: another channel (the old picture goes at once), not the same one again after a settings change
JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_watch(JNIEnv *, jclass, jdouble hz, jboolean fresh) {
  if (g_radio) g_radio->watch(hz);
  if (fresh && g_screen) g_screen->clear();
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_idle(JNIEnv *, jclass) {
  if (g_radio) g_radio->idle();
}

// The Spectrum tab: no picture while it sweeps, so the preview shows the channel's card (as in a scan), not a
// frame that looks live.
JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_sweep(JNIEnv *, jclass, jdouble lo_hz, jdouble hi_hz) {
  if (g_radio) g_radio->sweep(lo_hz, hi_hz);
  if (g_screen) g_screen->clear();
}

JNIEXPORT jstring JNICALL Java_com_fpvdroid_app_Native_spectrum(JNIEnv *env, jclass, jdouble lo_hz, jdouble hi_hz) {
  const std::string s = g_radio ? g_radio->spectrum_json(lo_hz, hi_hz) : std::string("{}");
  return env->NewStringUTF(s.c_str());
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_streamTest(JNIEnv *, jclass, jint rate_sel, jdouble seconds) {
  if (g_radio) g_radio->stream_test(rate_sel, seconds);
}

// The setters that return a jboolean: true when the radio says a watch must be started again to have the change.
JNIEXPORT jboolean JNICALL Java_com_fpvdroid_app_Native_setWatchRate(JNIEnv *, jclass, jint rate_sel) {
  return g_radio && g_radio->set_watch_rate(rate_sel);
}

JNIEXPORT jboolean JNICALL Java_com_fpvdroid_app_Native_setGain(JNIEnv *, jclass, jdouble db) {
  return g_radio && g_radio->set_gain(db);
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setDiversity(JNIEnv *, jclass, jboolean on) {
  if (g_radio) g_radio->set_combine(on);
}

JNIEXPORT jboolean JNICALL Java_com_fpvdroid_app_Native_setOffsetTuning(JNIEnv *, jclass, jboolean on) {
  return g_radio && g_radio->set_offset_tuning(on);
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setPicture(JNIEnv *, jclass, jboolean luma_comb, jboolean adaptive,
                                                             jboolean smooth, jboolean repair, jboolean colour_comb,
                                                             jboolean strips) {
  PictureOpts p;
  p.luma_comb = luma_comb;
  p.adaptive_comb = adaptive;
  p.smooth_lines = smooth;
  p.repair = repair;
  p.chroma_comb = colour_comb;
  p.strips = strips;
  if (g_radio) g_radio->set_picture(p);
  // ... and the screen draws the strips straight into the buffer the display shows, where the phone can
  if (g_screen) g_screen->set_front_buffer(strips);
}

// The picture's delay on screen (render/latency.h), over the last second, as JSON.
JNIEXPORT jstring JNICALL Java_com_fpvdroid_app_Native_latency(JNIEnv *env, jclass) {
  const std::string s = g_screen ? g_screen->latency_json() : std::string("{}");
  return env->NewStringUTF(s.c_str());
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setView(JNIEnv *, jclass, jfloat aspect, jint fit, jfloat scale,
                                                          jboolean stereo, jfloat eye_shift, jfloat offset_x,
                                                          jfloat offset_y) {
  if (!g_screen) return;
  ViewOpts v;
  v.aspect = aspect > 0 ? aspect : 4.f / 3;
  v.fit = fit == 1 ? Fit::Stretch : fit == 2 ? Fit::Crop : Fit::Fit;
  v.scale = std::clamp(scale, 0.2f, 3.f);
  v.stereo = stereo;
  v.eye_shift = std::clamp(eye_shift, -0.25f, 0.25f);
  v.offset_x = std::clamp(offset_x, -0.5f, 0.5f);
  v.offset_y = std::clamp(offset_y, -0.5f, 0.5f);
  g_screen->set_view(v);
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setBackdrop(JNIEnv *, jclass, jboolean on, jboolean now) {
  if (g_screen) g_screen->set_backdrop(on, now);
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setCardLabel(JNIEnv *env, jclass, jstring label) {
  if (g_screen) g_screen->set_card_label(jstring_of(env, label));
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setPreview(JNIEnv *, jclass, jint left, jint top, jint right,
                                                             jint bottom, jfloat radius) {
  if (!g_screen) return;
  const float w = static_cast<float>(std::max(0, right - left)), h = static_cast<float>(std::max(0, bottom - top));
  g_screen->set_preview(Preview{Rect{static_cast<float>(left), static_cast<float>(top), w, h}, std::max(0.f, radius)});
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_record(JNIEnv *env, jclass, jdouble seconds, jstring prefix) {
  const std::string p = jstring_of(env, prefix);
  if (g_radio) g_radio->record(seconds, p);
}

JNIEXPORT jstring JNICALL Java_com_fpvdroid_app_Native_status(JNIEnv *env, jclass) {
  const std::string s = g_radio ? g_radio->status_json() : std::string("{\"open\":false}");
  return env->NewStringUTF(s.c_str());
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setSurface(JNIEnv *env, jclass, jobject surface) {
  if (!g_screen) return;
  g_screen->set_window(surface ? ANativeWindow_fromSurface(env, surface) : nullptr);
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_redraw(JNIEnv *, jclass) {
  if (g_screen) g_screen->redraw();
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setDisplayRefresh(JNIEnv *, jclass, jfloat hz) {
  if (g_screen) g_screen->set_display_hz(hz);
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_testCard(JNIEnv *, jclass, jboolean on) {
  KP_LOGI("test picture %s", on ? "on" : "off");
  if (on) {
    card_start();
  } else {
    card_stop();
    if (g_screen) g_screen->clear();
  }
}

JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_log(JNIEnv *env, jclass, jstring msg) {
  KP_LOGI("app: %s", jstring_of(env, msg).c_str());
}

// Keep the picture on screen (dimmed once stale) for up to `ms`, through the close and reopen of a wedged adapter.
JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_holdPicture(JNIEnv *, jclass, jint ms) {
  if (g_screen) g_screen->hold(ms);
}

// Debug: a switch by name. The radio's own two are set at once ("0" turns one off; anything else, or nothing, on),
// and so are the screen's: KP_FRONT_BUFFER, KP_PRESENT_ALIGN, KP_PRESENT_MARGIN_US (render/screen.h).
// Any other is an environment variable for wifi-sdr / devourer, in place from the next open; an empty value
// removes it.
JNIEXPORT void JNICALL Java_com_fpvdroid_app_Native_setDebugEnv(JNIEnv *env, jclass, jstring key, jstring value) {
  const std::string k = jstring_of(env, key), v = jstring_of(env, value);
  if (k.empty()) return;
  const bool on = v.empty() || v[0] != '0';
  if (k == "KP_CONCEAL") {
    if (g_radio) g_radio->set_conceal(on);
  } else if (k == "KP_FAST_CPUS") {
    if (g_radio) g_radio->set_fast_cores(on);
  } else if (k == "KP_FRONT_BUFFER") {  // "0": no front-buffer drawing with Vsync off (Screen::allow_front_buffer)
    if (g_screen) g_screen->allow_front_buffer(on);
  } else if (k == "KP_PRESENT_ALIGN") {  // how strips go up (Screen::set_present_align, set_present_margin_us)
    if (g_screen) g_screen->set_present_align(on);
  } else if (k == "KP_PRESENT_MARGIN_US") {
    if (g_screen) g_screen->set_present_margin_us(std::atoi(v.c_str()));
  } else {
    std::lock_guard<std::mutex> l(g_env_m);
    g_env_wanted.emplace_back(k, v);
  }
  KP_LOGI("debug: env %s=%s", k.c_str(), v.c_str());
}

}  // extern "C"
