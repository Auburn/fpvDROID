package com.fpvdroid.app

import android.view.Surface

/**
 * What the app asks of its native side: the radio and the screen. In the app this is [Native]; a test, which
 * cannot load the library, puts a stand-in in [Engine.standIn] before the activity starts.
 */
interface Engine {
    fun init(logPath: String)
    /** USB 2 -> 3 switch on this fd: "requested" means the adapter is re-enumerating now. */
    fun usb3Switch(fd: Int): String
    /** Blocking (~5 s). Returns "" on success, else a message. */
    fun open(fd: Int): String
    fun close()
    fun superspeed(): Boolean
    /** Sweep the bands for video; then watch [preferHz] if it was found (0: the channel watched before), else the strongest. */
    fun scan(preferHz: Double)
    /** [fresh]: another channel (the old picture goes at once), not the same one again after a settings change. */
    fun watch(hz: Double, fresh: Boolean)
    /** Stop decoding (the adapter stays open). */
    fun idle()
    /** Sweep [loHz]..[hiHz] over and over for the spectrum page, until the next mode call. No picture meanwhile. */
    fun sweep(loHz: Double, hiHz: Double)
    /** What sweeps and scans have seen of [loHz]..[hiHz], as JSON (native/radio/radio.h, Radio::spectrum_json). */
    fun spectrum(loHz: Double, hiHz: Double): String
    fun streamTest(rateSel: Int, seconds: Double)
    // A setter that returns a Boolean sets something a watch only takes when it starts: true when that changed,
    // so that a channel being watched has to be watched again ([watch], not fresh) to have it.
    /** The rate watch uses (1 = 40 MS/s, 2 = 20 MS/s). A change also clears a 20 MS/s fallback. */
    fun setWatchRate(rateSel: Int): Boolean
    /** Gain: < 0 = the chip's AGC, else fixed gain (dB). */
    fun setGain(db: Double): Boolean
    /** Both antennas combined before demodulation. At once. */
    fun setDiversity(on: Boolean)
    /** Hardware centre beside the channel (off the chip's DC point). */
    fun setOffsetTuning(on: Boolean): Boolean
    /**
     * The decoder's picture switches (PictureOpts in native/dsp/video.h). At once. [colourComb]: the NTSC 3-line
     * chroma comb (fewer rainbows on fine detail, more decode CPU).
     */
    fun setPicture(lumaComb: Boolean, adaptiveComb: Boolean, smoothLines: Boolean, repair: Boolean, colourComb: Boolean)
    /** How the picture is laid out on the Surface. [fit]: 0 fit, 1 stretch, 2 crop. [eyeShift]: fraction of the width. */
    fun setView(aspect: Float, fit: Int, scale: Float, stereo: Boolean, eyeShift: Float)
    /**
     * The menu's backdrop: black, with the picture moved into the menu's preview ([setPreview]). The picture moves
     * there and back over a moment, unless [now]. Where there is no picture, NO SIGNAL takes its place.
     */
    fun setBackdrop(on: Boolean, now: Boolean)
    /** Where the menu's preview is on the Surface, in pixels, and its corner radius: where [setBackdrop] puts the picture. */
    fun setPreview(left: Int, top: Int, right: Int, bottom: Int, radius: Float)
    /** What the card shown where there is no picture says: the chosen channel ("R8"), or "" for NO SIGNAL. */
    fun setCardLabel(label: String)
    /** Debug: save the next [seconds] of watched IQ as [prefix].cs16 + .json. */
    fun record(seconds: Double, prefix: String)
    /** What the radio says of itself, as JSON: [RadioStatus.parse] reads it. */
    fun status(): String
    /** The Surface to draw on (null: it is going away; returns once nothing draws on the old one). */
    fun setSurface(surface: Surface?)
    /** Draw again (the Surface changed size). */
    fun redraw()
    /** Debug: a moving test picture in place of video (no adapter needed), for looking at the screen side alone. */
    fun testCard(on: Boolean) {}
    /**
     * Debug: a switch by name. KP_CONCEAL and KP_FAST_CPUS are the radio's own ("0" turns one off). Any other is an
     * environment variable for wifi-sdr / devourer, in place from the next open ("" removes it).
     */
    fun setDebugEnv(key: String, value: String) {}
    /** Keep the picture on screen, dimmed once stale, for up to [ms] (through a close and reopen); the next frame ends it. */
    fun holdPicture(ms: Int) {}
    fun log(msg: String)

    companion object {
        @Volatile var standIn: Engine? = null
        fun get(): Engine = standIn ?: Native
    }
}

/** The native radio and screen (native/jni/fpvdroid_jni.cpp). */
object Native : Engine {
    init { System.loadLibrary("fpvdroid") }

    external override fun init(logPath: String)
    external override fun usb3Switch(fd: Int): String
    external override fun open(fd: Int): String
    external override fun close()
    external override fun superspeed(): Boolean
    external override fun scan(preferHz: Double)
    external override fun watch(hz: Double, fresh: Boolean)
    external override fun idle()
    external override fun sweep(loHz: Double, hiHz: Double)
    external override fun spectrum(loHz: Double, hiHz: Double): String
    external override fun streamTest(rateSel: Int, seconds: Double)
    external override fun setWatchRate(rateSel: Int): Boolean
    external override fun setGain(db: Double): Boolean
    external override fun setDiversity(on: Boolean)
    external override fun setOffsetTuning(on: Boolean): Boolean
    external override fun setPicture(lumaComb: Boolean, adaptiveComb: Boolean, smoothLines: Boolean, repair: Boolean, colourComb: Boolean)
    external override fun setView(aspect: Float, fit: Int, scale: Float, stereo: Boolean, eyeShift: Float)
    external override fun setBackdrop(on: Boolean, now: Boolean)
    external override fun setPreview(left: Int, top: Int, right: Int, bottom: Int, radius: Float)
    external override fun setCardLabel(label: String)
    external override fun record(seconds: Double, prefix: String)
    external override fun status(): String
    external override fun setSurface(surface: Surface?)
    external override fun redraw()
    external override fun testCard(on: Boolean)
    external override fun setDebugEnv(key: String, value: String)
    external override fun holdPicture(ms: Int)
    external override fun log(msg: String)
}
