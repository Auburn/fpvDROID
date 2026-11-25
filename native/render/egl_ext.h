// fpvDROID screen: whether the display's EGL has an extension.
#pragma once
#include <EGL/egl.h>

#include <cstring>

namespace kp {

// `name` is one of the names in the display's list of extensions (a whole name, not the start of a longer one).
inline bool egl_has(EGLDisplay dpy, const char *name) {
  const char *all = eglQueryString(dpy, EGL_EXTENSIONS);
  const size_t n = std::strlen(name);
  for (const char *p = all; p && (p = std::strstr(p, name)); p += n)
    if ((p == all || p[-1] == ' ') && (p[n] == ' ' || p[n] == 0)) return true;
  return false;
}

}  // namespace kp
