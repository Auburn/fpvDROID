package com.fpvdroid.app

import org.json.JSONException
import org.json.JSONObject

/**
 * Something on the air: a hump in the spectrum (any kind of transmitter, not only analog video). [kind]: that of the
 * [Signal] the radio has recognised it to be on, or null (not recognised: analog, or not judged).
 */
data class Carrier(val mhz: Double, val widthMhz: Double, val snrDb: Float, val kind: SignalKind? = null)

/**
 * What the radio recognises on the air by itself while it sweeps (Radio::spectrum_json's "kind" of a signal, and of the
 * carriers on it; native/radio/radio.h): digital video by its system (native/dsp/digital.h), DJI by its 2.5 ms TDD
 * frame, HDZero by a flat block sent without a break on one of its channels, 27 MHz wide or 17 in its narrow mode, and
 * any other flat, wide one as plain digital. [label]: what the Spectrum tab calls it, before the channel's name.
 */
enum class SignalKind(val code: String, val label: String) {
    DJI("dji", "DJI"),
    HDZERO("hdzero", "HDZero"),
    HDZERO_NARROW("hdzero_narrow", "HDZero narrow"),
    DIGITAL("digital", "Digital");

    companion object {
        /** The kind the radio named, or null (none, or one this app does not know). */
        fun of(code: String?): SignalKind? = entries.firstOrNull { it.code == code }
    }
}

/**
 * Something the radio has recognised on the air by itself (Radio::spectrum_json's "signals"): what [kind] of signal
 * takes the band from [loMhz] to [hiMhz], and what the radio calls it ([label]: "DJI", "HDZero narrow"). [active]: in
 * use; false: on the air but idle. A digital video link is always active: it sends all the time it is on.
 */
data class Signal(val loMhz: Double, val hiMhz: Double, val kind: SignalKind, val label: String, val active: Boolean = true)

/**
 * What the radio has seen of the bands (native/radio/radio.h, Radio::spectrum_json): [db] is dB over the noise floor
 * from [firstMhz] in steps of [binMhz], NaN where nothing has been seen yet. [atMhz]: the middle of the stretch being
 * swept right now ([atWidthMhz] wide), 0 when the radio is not sweeping. [sweeps]: passes done since the sweep began.
 * [signals]: what the radio has recognised there, a carrier on one tagged with its kind ([Carrier.kind]).
 */
class Spectrum(
    val firstMhz: Double, val binMhz: Double, val db: FloatArray, val carriers: List<Carrier>,
    val atMhz: Double = 0.0, val atWidthMhz: Double = 33.0, val sweeps: Int = 0, val signals: List<Signal> = emptyList(),
) {
    val empty: Boolean get() = db.none { !it.isNaN() }

    companion object {
        /** Null for anything that is not the radio's spectrum (an empty object while it has no radio, a broken string). */
        fun parse(json: String): Spectrum? = try {
            val o = JSONObject(json)
            val arr = o.optJSONArray("db")
            val bin = o.optDouble("bin_mhz", 0.0)
            if (arr == null || !(bin > 0)) null else Spectrum(
                firstMhz = o.optDouble("first_mhz", 0.0),
                binMhz = bin,
                db = FloatArray(arr.length()) { i -> if (arr.isNull(i)) Float.NaN else arr.optDouble(i, Double.NaN).toFloat() },
                carriers = buildList {
                    val cs = o.optJSONArray("carriers") ?: return@buildList
                    for (i in 0 until cs.length()) {
                        val c = cs.optJSONObject(i) ?: continue
                        val mhz = c.optDouble("mhz", Double.NaN)
                        if (mhz.isFinite() && mhz > 0) add(Carrier(mhz, c.optDouble("width", 0.0), c.optDouble("snr", 0.0).toFloat(),
                            SignalKind.of(c.optString("kind"))))
                    }
                },
                atMhz = o.optDouble("at_mhz", 0.0).let { if (it.isFinite()) it else 0.0 },
                atWidthMhz = o.optDouble("at_width_mhz", 33.0),
                sweeps = o.optInt("sweeps"),
                signals = buildList {
                    val ss = o.optJSONArray("signals") ?: return@buildList
                    for (i in 0 until ss.length()) {
                        val s = ss.optJSONObject(i) ?: continue
                        val kind = SignalKind.of(s.optString("kind")) ?: continue   // one this app does not know yet
                        val lo = s.optDouble("lo_mhz", Double.NaN)
                        val hi = s.optDouble("hi_mhz", Double.NaN)
                        if (lo.isFinite() && hi.isFinite() && hi > lo)
                            add(Signal(lo, hi, kind, s.optString("label", kind.label), s.optBoolean("active", true)))
                    }
                },
            )
        } catch (e: JSONException) {
            null
        }
    }
}

/** Something on the air ([carrier]), where the Spectrum tab puts it ([spot]: whose channel, what picking it tunes). */
data class OnAir(val carrier: Carrier, val spot: Spot)
