cmake_minimum_required(VERSION 3.22)
project(fpvdroid C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(FPVDROID_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
option(FPVDROID_RADIO "Build the radio (wifi-sdr + devourer + libusb)" ON)

# ---- DSP: no hardware, shared by the app and the desktop tools ----
add_library(fpvdroid_dsp STATIC dsp/video.cpp dsp/deinterlace.cpp dsp/line_draw.cpp dsp/picture_filters.cpp
            dsp/sparkle.cpp dsp/spectrum.cpp dsp/log.cpp)
# the Spectrum tab's Wi-Fi networks (web/CMakeLists.txt lists it too)
target_sources(fpvdroid_dsp PRIVATE dsp/wifi.cpp)
target_include_directories(fpvdroid_dsp PUBLIC dsp)
target_compile_options(fpvdroid_dsp PRIVATE -O3 -ffast-math -Wall -Wextra)
if(ANDROID)
  target_link_libraries(fpvdroid_dsp PUBLIC log)
endif()

# ---- the picture on screen: an OpenGL ES 3 renderer (the app's Surface; off screen in kp_render) ----
if(ANDROID)
  # ... and what the app's screen (render/screen.h) has besides: the display's timing, front-buffer drawing
  add_library(fpvdroid_render STATIC render/video_renderer.cpp render/no_signal.cpp render/latency.cpp
              render/display_timing.cpp render/front_buffer.cpp)
  target_link_libraries(fpvdroid_render PUBLIC fpvdroid_dsp GLESv3 EGL)
else()
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(KP_GLES IMPORTED_TARGET QUIET egl glesv2)  # e.g. Mesa: libegl-dev libgles-dev
  endif()
  if(KP_GLES_FOUND)
    add_library(fpvdroid_render STATIC render/video_renderer.cpp render/no_signal.cpp render/latency.cpp)
    target_link_libraries(fpvdroid_render PUBLIC fpvdroid_dsp PkgConfig::KP_GLES)
  endif()
endif()
if(TARGET fpvdroid_render)
  target_include_directories(fpvdroid_render PUBLIC render)
  target_compile_options(fpvdroid_render PRIVATE -O2 -Wall -Wextra)
endif()

# ---- radio ----
if(FPVDROID_RADIO)
  if(ANDROID)
    include(cmake/libusb_android.cmake)  # libusb from source + a pkg-config file devourer can find
  endif()
  # wifi-sdr assembles the on-chip streamer with mipsel binutils (shell.nix: FPVDROID_MIPS_PREFIX)
  if(DEFINED ENV{FPVDROID_MIPS_PREFIX})
    set(MIPS_AS "$ENV{FPVDROID_MIPS_PREFIX}as" CACHE FILEPATH "")
    set(MIPS_LD "$ENV{FPVDROID_MIPS_PREFIX}ld" CACHE FILEPATH "")
    set(MIPS_OBJCOPY "$ENV{FPVDROID_MIPS_PREFIX}objcopy" CACHE FILEPATH "")
  endif()
  include(cmake/patches.cmake)  # Play RX's wifi-sdr + devourer patches (patches/), applied once
  add_subdirectory(${FPVDROID_ROOT}/third_party/wifi-sdr wifi-sdr EXCLUDE_FROM_ALL)
  if(ANDROID)
    target_link_libraries(devourer PUBLIC usb-1.0-static)
  endif()
  # web/CMakeLists.txt lists the same sources
  set(FPVDROID_RADIO_SOURCES radio/radio.cpp radio/combine.cpp radio/decode_queue.cpp radio/fallback_policy.cpp
      radio/frame_gate.cpp radio/steps.cpp radio/sweep.cpp radio/thread_placement.cpp)
  add_library(fpvdroid_radio STATIC ${FPVDROID_RADIO_SOURCES})
  target_include_directories(fpvdroid_radio PUBLIC radio)
  target_link_libraries(fpvdroid_radio PUBLIC fpvdroid_dsp wsdrcore)
  target_compile_options(fpvdroid_radio PRIVATE -O2 -Wall -Wextra)
endif()

# ---- desktop tools ----
if(NOT ANDROID)
  find_package(Threads REQUIRED)
  add_executable(kp_replay tools/kp_replay.cpp)
  target_link_libraries(kp_replay PRIVATE fpvdroid_dsp)
  target_compile_options(kp_replay PRIVATE -O2 -Wall)
  add_executable(kp_gen tools/kp_gen.cpp)  # a synthetic transmitter: recordings to test with, no hardware
  target_link_libraries(kp_gen PRIVATE fpvdroid_dsp)
  target_compile_options(kp_gen PRIVATE -O2 -Wall)
  if(TARGET fpvdroid_render)
    add_executable(kp_render tools/kp_render.cpp)  # the app's renderer off screen (needs EGL + OpenGL ES 3)
    target_link_libraries(kp_render PRIVATE fpvdroid_render)
    target_compile_options(kp_render PRIVATE -O2 -Wall)
  endif()
  if(FPVDROID_RADIO)
    add_executable(kp_live tools/kp_live.cpp)
    target_link_libraries(kp_live PRIVATE fpvdroid_radio Threads::Threads)
    target_compile_options(kp_live PRIVATE -O2 -Wall)
    # kp_live with a simulated receiver that plays a recording back as a tunable adapter (KP_SIM_FILE):
    # the scan -> pick -> watch flow end to end without hardware. sim_receiver.o supplies every
    # wsdr::Receiver symbol, so the archive's receiver.o is never pulled in.
    add_executable(kp_sim tools/kp_live.cpp ${FPVDROID_RADIO_SOURCES} tools/sim_receiver.cpp)
    target_include_directories(kp_sim PRIVATE radio)
    target_link_libraries(kp_sim PRIVATE fpvdroid_dsp wsdrcore Threads::Threads)
    target_compile_options(kp_sim PRIVATE -O2 -Wall)
  endif()
endif()

# ---- unit tests (desktop only): ctest --test-dir <build dir> ----
if(NOT ANDROID)
  enable_testing()
  # the radio's rules that need no adapter: the sources themselves, so this builds with FPVDROID_RADIO off too
  add_executable(kp_radio_tests tests/radio_tests.cpp radio/combine.cpp radio/decode_queue.cpp
                 radio/fallback_policy.cpp radio/frame_gate.cpp radio/steps.cpp)
  target_include_directories(kp_radio_tests PRIVATE radio tests)
  target_link_libraries(kp_radio_tests PRIVATE fpvdroid_dsp Threads::Threads)
  target_compile_options(kp_radio_tests PRIVATE -O2 -Wall -Wextra)
  add_test(NAME radio COMMAND kp_radio_tests)
  # the decoder's sync tracking, on a signal the test makes itself
  add_executable(kp_decoder_tests tests/decoder_tests.cpp)
  target_link_libraries(kp_decoder_tests PRIVATE fpvdroid_dsp)
  target_compile_options(kp_decoder_tests PRIVATE -O2 -Wall -Wextra)
  add_test(NAME decoder COMMAND kp_decoder_tests)
  # the Spectrum tab's band picture, from sweep steps the test makes itself (on the radio's own plan)
  add_executable(kp_spectrum_tests tests/spectrum_tests.cpp radio/sweep.cpp)
  target_include_directories(kp_spectrum_tests PRIVATE radio)
  target_link_libraries(kp_spectrum_tests PRIVATE fpvdroid_dsp)
  target_compile_options(kp_spectrum_tests PRIVATE -O2 -Wall -Wextra)
  add_test(NAME spectrum COMMAND kp_spectrum_tests)
  # the Spectrum tab's Wi-Fi networks, from sweep steps the test makes itself as raw IQ
  add_executable(kp_wifi_tests tests/wifi_tests.cpp radio/sweep.cpp)
  target_include_directories(kp_wifi_tests PRIVATE radio)
  target_link_libraries(kp_wifi_tests PRIVATE fpvdroid_dsp)
  target_compile_options(kp_wifi_tests PRIVATE -O2 -Wall -Wextra)
  add_test(NAME wifi COMMAND kp_wifi_tests)
  # the screen's picture delay (render/latency.h: no GL in it)
  add_executable(kp_screen_tests tests/screen_tests.cpp render/latency.cpp)
  target_include_directories(kp_screen_tests PRIVATE render)
  target_compile_options(kp_screen_tests PRIVATE -O2 -Wall -Wextra)
  add_test(NAME screen COMMAND kp_screen_tests)
endif()

# ---- Android JNI ----
if(ANDROID)
  add_library(fpvdroid SHARED jni/fpvdroid_jni.cpp)
  target_link_libraries(fpvdroid PRIVATE fpvdroid_radio fpvdroid_render android log)
  target_compile_options(fpvdroid PRIVATE -O2 -Wall)
endif()
