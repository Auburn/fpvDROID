// fpvDROID screen: front-buffer drawing. See front_buffer.h.
#include "front_buffer.h"

#include <EGL/eglext.h>
#include <GLES3/gl3.h>

#include <algorithm>

#include "egl_ext.h"
#include "log.h"

namespace kp {

namespace {
constexpr char kMutable[] = "EGL_KHR_mutable_render_buffer";
constexpr char kAutoRefresh[] = "EGL_ANDROID_front_buffer_auto_refresh";
}  // namespace

bool FrontBuffer::config_can(EGLDisplay dpy, EGLConfig cfg) {
  EGLint type = 0;
  return eglGetConfigAttrib(dpy, cfg, EGL_SURFACE_TYPE, &type) && (type & EGL_MUTABLE_RENDER_BUFFER_BIT_KHR);
}

bool FrontBuffer::wants_config(EGLDisplay dpy) const { return _asked && egl_has(dpy, kMutable); }

bool FrontBuffer::start(EGLDisplay dpy, EGLSurface surf, EGLContext ctx, EGLConfig cfg) {
  _on = false;
  if (!_asked) return false;
  const bool has = egl_has(dpy, kMutable);
  if (!has || !config_can(dpy, cfg)) {
    KP_LOGW("screen: front-buffer drawing asked for, but %s: the picture goes up as without it",
            has ? "the EGL config cannot be single-buffered" : "EGL has no EGL_KHR_mutable_render_buffer");
    return false;
  }
  if (!eglSurfaceAttrib(dpy, surf, EGL_RENDER_BUFFER, EGL_SINGLE_BUFFER)) {
    KP_LOGW("screen: front-buffer drawing: the surface cannot be single-buffered (0x%x)", eglGetError());
    return false;
  }
  const bool refresh =
      egl_has(dpy, kAutoRefresh) && eglSurfaceAttrib(dpy, surf, EGL_FRONT_BUFFER_AUTO_REFRESH_ANDROID, EGL_TRUE);
  // single-buffered from the next swap on: of a black screen
  EGLint w = 0, h = 0;
  eglQuerySurface(dpy, surf, EGL_WIDTH, &w);
  eglQuerySurface(dpy, surf, EGL_HEIGHT, &h);
  glDisable(GL_SCISSOR_TEST);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, std::max(w, 1), std::max(h, 1));
  glClearColor(0, 0, 0, 1);
  glClear(GL_COLOR_BUFFER_BIT);
  eglSwapBuffers(dpy, surf);
  EGLint buffer = 0;
  eglQueryContext(dpy, ctx, EGL_RENDER_BUFFER, &buffer);
  _on = buffer == EGL_SINGLE_BUFFER;
  KP_LOGI("screen: front-buffer drawing %s (compositor auto refresh %s)",
          _on ? "on" : "asked for, but the surface stayed double-buffered", refresh ? "on" : "off");
  return _on;
}

bool FrontBuffer::wait_drawn(int timeout_ms) {
  GLsync s = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
  if (!s) {
    glFinish();
    return true;
  }
  const GLenum r = glClientWaitSync(s, GL_SYNC_FLUSH_COMMANDS_BIT, static_cast<GLuint64>(timeout_ms) * 1000000);
  glDeleteSync(s);
  return r == GL_ALREADY_SIGNALED || r == GL_CONDITION_SATISFIED;
}

}  // namespace kp
