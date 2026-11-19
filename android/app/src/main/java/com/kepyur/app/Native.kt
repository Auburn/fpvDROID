package com.kepyur.app

import android.view.Surface

/** The native radio and screen (native/jni/kepyur_jni.cpp). */
object Native {
    init { System.loadLibrary("kepyur") }

    @JvmStatic external fun init(logPath: String)
    /** USB 2 -> 3 switch on this fd: "requested" means the adapter is re-enumerating now. */
    @JvmStatic external fun usb3Switch(fd: Int): String
    /** Blocking (~5 s). Returns "" on success, else a message. */
    @JvmStatic external fun open(fd: Int): String
    @JvmStatic external fun close()
    @JvmStatic external fun superspeed(): Boolean
    /** Sweep the bands for video; then watch [preferHz] if it was found (0: the channel watched before), else the strongest. */
    @JvmStatic external fun scan(preferHz: Double)
    /** [fresh]: another channel (the old picture goes at once), not the same one again after a settings change. */
    @JvmStatic external fun watch(hz: Double, fresh: Boolean)
    /** Stop decoding (the adapter stays open). */
    @JvmStatic external fun idle()
    @JvmStatic external fun streamTest(rateSel: Int, seconds: Double)
    /** The rate watch uses (1 = 40 MS/s, 2 = 20 MS/s), from the next watch on. Also clears a 20 MS/s fallback. */
    @JvmStatic external fun setWatchRate(rateSel: Int)
    /** Debug: gain from the next tune on: < 0 = the chip's AGC, else fixed gain (dB). */
    @JvmStatic external fun setGain(db: Double)
    /** NTSC 3-line chroma comb (fewer rainbows on fine detail, more decode CPU), from the next watch on. */
    @JvmStatic external fun setComb(on: Boolean)
    /** Both antennas combined before demodulation. At once. */
    @JvmStatic external fun setDiversity(on: Boolean)
    /** Hardware centre beside the channel (off the chip's DC point). True when it changed: watch again to apply. */
    @JvmStatic external fun setOffsetTuning(on: Boolean): Boolean
    /** The decoder's picture switches (native/dsp/video.h). At once. */
    @JvmStatic external fun setPicture(lumaComb: Boolean, adaptiveComb: Boolean, smoothLines: Boolean, repair: Boolean)
    /** How the picture is laid out on the Surface. [fit]: 0 fit, 1 stretch, 2 crop. [eyeShift]: fraction of the width. */
    @JvmStatic external fun setView(aspect: Float, fit: Int, scale: Float, stereo: Boolean, eyeShift: Float)
    /** The menu's backdrop: the picture blurred and dimmed, eased in and out unless [now]. */
    @JvmStatic external fun setBackdrop(on: Boolean, now: Boolean)
    /** Debug: save the next [seconds] of watched IQ as [prefix].cs16 + .json. */
    @JvmStatic external fun record(seconds: Double, prefix: String)
    @JvmStatic external fun status(): String
    /** The Surface to draw on (null: it is going away; returns once nothing draws on the old one). */
    @JvmStatic external fun setSurface(surface: Surface?)
    /** Draw again (the Surface changed size). */
    @JvmStatic external fun redraw()
    @JvmStatic external fun log(msg: String)
}
