// fpvDROID: the picture on screen (OpenGL ES 3). Draws the decoder's frames into a viewport: laid out by aspect
// ratio, fit mode and scale, once or twice side by side (goggles that hold a phone), or small in the menu's
// preview with black all round it. Where there is no picture, the NO SIGNAL card (no_signal.h) takes its place.
//
// No EGL here: the caller makes a context current (the app: native/jni, on its render thread; the bench:
// native/tools/kp_render.cpp, off screen) and calls init(), then upload() / draw(), all on that one thread.
#pragma once
#include <cstdint>
#include <string>

namespace kp {

enum class Fit {
  Fit = 0,      // the whole picture, as large as fits its window (black bars where the shapes differ)
  Stretch = 1,  // the picture pulled to the window's shape
  Crop = 2,     // the window filled, the picture's overflow cut off
};

// A rectangle in pixels from the viewport's top left.
struct Rect {
  float x, y, w, h;
  bool operator==(const Rect &) const = default;
};

// The view settings: how the picture is laid out on the screen.
struct ViewOpts {
  float aspect = 4.f / 3;  // the shape the picture is shown at: 4:3, or 16:9 for a widescreen camera
  Fit fit = Fit::Fit;
  float scale = 1;         // the picture's size against what `fit` gives, about its window's centre
  bool stereo = false;     // two windows, the left and right halves of the viewport, the same picture in both
  // stereo only (lens spacing, and where the phone sits, differ between goggles):
  float eye_shift = 0;     // each picture moved toward the middle of the viewport (+) or away (-), as a fraction of
                           // the viewport's width
  float offset_x = 0;      // both windows and their pictures moved right (+) or left (-), a fraction of the width
  float offset_y = 0;      // both moved up (+) or down (-), a fraction of the viewport's height
  bool operator==(const ViewOpts &) const = default;
};

// The menu's preview: where it is on the viewport, and its corners' radius (pixels).
struct Preview {
  Rect box{0, 0, 0, 0};
  float radius = 0;
  bool operator==(const Preview &) const = default;
};

// What the screen decides itself, frame by frame, on top of the view settings: the picture's way into the menu's
// preview, its fade into the NO SIGNAL card, and what the card says.
struct Scene {
  Preview preview;
  float inset = 0;     // 0 = the picture where the view settings put it ... 1 = in the preview, laid out there by
                       // aspect and fit alone, black all round; in between it moves. The card is dimmed there,
                       // under the menu's status.
  float gain = 1;      // how much of the picture: 1 = all of it, 0 = none (the NO SIGNAL card in its place, e.g. a
                       // stale picture faded out), in between the two cross-faded
  float bright = 1;    // the picture's own brightness (a stale one, held over a reopen, is dimmed)
  std::string label;   // what the card says: the chosen channel ("R8"), or "" for NO SIGNAL
  bool operator==(const Scene &) const = default;
};

// Where a picture lands in the window `win`, both in pixels from the viewport's top left (it may reach outside
// the window: Crop, scale > 1). Pure arithmetic.
Rect layout(const Rect &win, float aspect, Fit fit, float scale);

class VideoRenderer {
public:
  // With a current ES 3 context. False (and a log line) if a shader does not build.
  bool init();
  // Frees the GL objects (context current), or forgets them (context already gone).
  void destroy(bool context_alive = true);
  // A new frame: 4 bytes per pixel, R G B X, top row first (kp::Frame). Only its rows [y0, y1) have changed since
  // the last upload (y1 < 0: to the bottom; low latency, PictureOpts::strips): only those go to the GPU, unless the
  // frame's size has changed too.
  void upload(const uint8_t *rgbx, int width, int height, int y0 = 0, int y1 = -1);
  void clear();  // no picture: the NO SIGNAL card until the next upload
  bool has_picture() const { return _have; }
  // The whole viewport, into framebuffer `target` (0 = the window surface).
  void draw(unsigned target, int view_w, int view_h, const ViewOpts &view, const Scene &scene);
  // Front-buffer drawing (front_buffer.h), where what was drawn before is still on the screen: only the part of
  // the viewport that shows the picture's rows [y0, y1), and nothing cleared. Only where that is all that can have
  // changed: the last draw() was into the same target at the same size, with the same view and scene, and of the
  // picture alone at its full size (no card, no fade; not in the preview, whose rounded corners are blended into
  // what is under them). False, and nothing drawn: it was not. Then draw().
  bool draw_band(unsigned target, int view_w, int view_h, const ViewOpts &view, const Scene &scene, int y0, int y1);
  // What draw() drew is not there any more (a new surface): draw_band() refuses until the next draw().
  void forget_drawn() { _drawn.any = false; }

private:
  void ensure_card(float aspect, const std::string &label);
  void quad(unsigned tex, int tex_w, int tex_h, float gain, float alpha, bool bicubic);
  // draw() and draw_band(): the scene, the whole viewport (cleared first) or only what shows the picture's rows
  // between band[0] and band[1] (fractions of its height, top first).
  void paint(unsigned target, int view_w, int view_h, const ViewOpts &view, const Scene &scene, const float *band);

  // the last draw()
  struct Drawn {
    bool any = false;      // there was one (forget_drawn)
    bool picture = false;  // ... of the picture alone, at its full size
    unsigned target = 0;
    int w = 0, h = 0;
    ViewOpts view;
    Scene scene;
  } _drawn;
  unsigned _prog = 0;
  unsigned _vbo = 0, _vao = 0, _tex = 0, _card = 0;
  int _tex_w = 0, _tex_h = 0, _card_w = 0, _card_h = 0;
  std::string _card_label;  // what the card made says
  bool _have = false;
  // uniform locations
  int _u_rect = -1, _u_size = -1, _u_gain = -1, _u_alpha = -1, _u_bicubic = -1, _u_clip = -1, _u_radius = -1;
};

}  // namespace kp
