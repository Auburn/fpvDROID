package com.fpvdroid.app

import org.json.JSONObject

/**
 * What the radio said at one status poll: the JSON of Radio::status_json (native/radio/radio.cpp), read once.
 * Its keys are named here and nowhere else in the app.
 *
 * The native side prints doubles as they come, and Android's JSON reader takes a bare nan or inf for a string
 * without complaint: a number that is missing or not finite reads as 0 here.
 */
class RadioStatus private constructor(s: JSONObject) {
    /** What the radio is doing. */
    enum class Mode { IDLE, SCAN, WATCH, TEST, SWEEP }

    /** The decoder's view of the picture. */
    class Video(o: JSONObject) {
        val locked = o.optBoolean("locked")
        val std: String = o.optString("std")   // "NTSC" / "PAL"
        val colour = o.optBoolean("colour")
        val fps = o.num("fps")
    }

    /** The two antennas, each alone and combined (dB). */
    class Antennas(o: JSONObject) {
        val combine = o.optBoolean("combine")
        val snrA = o.num("snr_a")
        val snrB = o.num("snr_b")
        val snrC = o.num("snr_c")
    }

    /** A stream test: how far it has got, or with [done] its result. */
    class Test(o: JSONObject) {
        val done = o.optBoolean("done")
        val rateMsps = o.num("rate_msps")
        val seconds = o.num("seconds")
        val banks = o.optLong("banks")
        val expected = o.optLong("expected")
        val seqGaps = o.optLong("seq_gaps")
        val restarts = o.optLong("restarts")
        /** All of it as the radio wrote it, for the log. */
        val json: String = o.toString()
    }

    /** A signal a scan has seen, and how many whole seconds ago the radio last saw it. */
    class Pilot(o: JSONObject) {
        val mhz = o.num("mhz")
        val name: String = o.optString("name")
        val snrDb = o.num("snr").toFloat()
        val std: String = o.optString("std")
        val ageS = o.optLong("age_s")
    }

    /** null: the status does not say (a status cut down to a few keys). */
    val mode: Mode? = when (s.optString("mode")) {
        "idle" -> Mode.IDLE
        "scan" -> Mode.SCAN
        "watch" -> Mode.WATCH
        "test" -> Mode.TEST
        "sweep" -> Mode.SWEEP
        else -> null
    }

    // the adapter and its stream
    val open = s.optBoolean("open", true)
    val usb3 = s.optBoolean("usb3")
    /** The receiver has ended: nothing more will come until the adapter is opened again. */
    val dead = s.optBoolean("dead")
    /** ... because the chip's USB data path wedged. */
    val wedged = s.optBoolean("wedged")
    val rxState: String = s.optString("rx_state")
    val rxError: String = s.optString("rx_error")
    val lastFault: String = s.optString("last_fault")
    /** Blocks of samples received since the open; -1: the status does not say. */
    val banks = s.optLong("banks", -1)
    val restarts = s.optLong("restarts")

    // blocks lost on the way: ones the adapter's stream gave up on (collected too late), ones dropped in the
    // queue behind the USB link, and ones dropped before the decoder
    val streamDropped = s.optLong("stream_dropped")
    val queueDropped = s.optLong("dropped")
    val decodeDropped = s.optLong("decode_dropped")
    /** How many of [streamDropped] the watch now running lost in its last few seconds (0 when not watching). */
    val lateRecent = s.optLong("late_recent")
    /** The busier of the two processing threads, 0..1 of its time. */
    val load = maxOf(s.num("dsp_load"), s.num("decode_load"))

    // the watch
    val watchHz = s.num("watch_hz")
    val watchMsps = s.num("watch_msps")
    /** 40 MS/s kept dropping out: watching at 20. */
    val fallback = s.optBoolean("fallback")
    /** null: not measured (not watching, or not yet). */
    val snrDb: Double? = if (s.has("snr_db")) s.num("snr_db") else null
    val video: Video? = s.optJSONObject("video")?.let { Video(it) }
    val antennas: Antennas? = s.optJSONObject("antennas")?.let { Antennas(it) }

    // the scan
    val scanStep = s.optInt("scan_step")
    val scanSteps = s.optInt("scan_steps")
    /** Every signal the radio has ever seen, not only the last scan's. */
    val pilots: List<Pilot> = s.optJSONArray("pilots")?.let { arr ->
        (0 until arr.length()).mapNotNull { i -> arr.optJSONObject(i)?.let { Pilot(it) } }
    } ?: emptyList()

    /** null until a stream test has run. */
    val test: Test? = s.optJSONObject("test")?.let { Test(it) }

    companion object {
        /** Throws [org.json.JSONException] for text that is not JSON at all (cut short, or something else). */
        fun parse(json: String) = RadioStatus(JSONObject(json))
    }
}

/**
 * How old the picture is on screen, over the last second or so: the JSON of latency_json
 * (native/render/latency.cpp), read once. Its keys are named here and nowhere else in the app.
 *
 * From the adapter taking a line to that line on the screen, measured as far along that way as the display lets it
 * be: each of the three is null where there is no such measurement.
 */
class PictureDelay private constructor(o: JSONObject) {
    /** A delay at the two ends of the picture: its top line and its bottom line. */
    class Ends(val topMs: Double, val bottomMs: Double)

    /** To the display showing it, as the display itself says. */
    val shown: Ends? = o.ends("shown")
    /**
     * ... as the compositor expected to show it, where the display gives no time for each frame. With [front]: to
     * it being drawn, plus half a refresh.
     */
    val expected: Ends? = o.ends("expected")
    /** To the frame being handed to the display. */
    val queued: Ends? = o.ends("queued")
    /** The display's refresh rate; 0: not known. */
    val refreshHz = o.num("refresh_hz")
    /** Front-buffer drawing (a debug switch): the picture is drawn straight into what the display shows. */
    val front = o.optBoolean("front")

    companion object {
        /** Throws [org.json.JSONException] for text that is not JSON at all. */
        fun parse(json: String) = PictureDelay(JSONObject(json))
    }
}

private fun JSONObject.num(key: String): Double = optDouble(key, 0.0).let { if (it.isFinite()) it else 0.0 }

/** The pair "<name>_top_ms", "<name>_bottom_ms"; null where there is no top (nothing measured). */
private fun JSONObject.ends(name: String): PictureDelay.Ends? =
    num("${name}_top_ms").takeIf { it > 0 }?.let { PictureDelay.Ends(it, num("${name}_bottom_ms")) }
