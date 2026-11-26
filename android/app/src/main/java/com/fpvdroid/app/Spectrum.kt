package com.fpvdroid.app

import org.json.JSONException
import org.json.JSONObject

/** Something on the air: a hump in the spectrum (any kind of transmitter, not only analog video). */
data class Carrier(val mhz: Double, val widthMhz: Double, val snrDb: Float)

/**
 * A 5 GHz Wi-Fi network the radio has seen in the last five minutes (native/dsp/wifi.h): from [loMhz] to [hiMhz], the
 * edges of the 20, 40, 80 or 160 MHz it was seen to light, named [label] ("Wi-Fi 149", "Wi-Fi 100–112"). [busy]: seen
 * in use; else it has only been heard sending its beacons, and may take its channels (and with 40 MHz or more, its
 * whole block) as soon as someone uses it.
 */
data class Wifi(val loMhz: Double, val hiMhz: Double, val label: String, val busy: Boolean) {
    val mhz: Double get() = (loMhz + hiMhz) / 2
}

/**
 * What the radio has seen of the bands (native/radio/radio.h, Radio::spectrum_json): [db] is dB over the noise floor
 * from [firstMhz] in steps of [binMhz], NaN where nothing has been seen yet. [atMhz]: the middle of the stretch being
 * swept right now ([atWidthMhz] wide), 0 when the radio is not sweeping. [sweeps]: passes done since the sweep began.
 * [wifi]: the Wi-Fi networks seen there, apart from [carriers] (a network in use is among those too).
 */
class Spectrum(
    val firstMhz: Double, val binMhz: Double, val db: FloatArray, val carriers: List<Carrier>,
    val atMhz: Double = 0.0, val atWidthMhz: Double = 33.0, val sweeps: Int = 0, val wifi: List<Wifi> = emptyList(),
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
                        if (mhz.isFinite() && mhz > 0) add(Carrier(mhz, c.optDouble("width", 0.0), c.optDouble("snr", 0.0).toFloat()))
                    }
                },
                atMhz = o.optDouble("at_mhz", 0.0).let { if (it.isFinite()) it else 0.0 },
                atWidthMhz = o.optDouble("at_width_mhz", 33.0),
                sweeps = o.optInt("sweeps"),
                wifi = buildList {
                    val ws = o.optJSONArray("wifi") ?: return@buildList
                    for (i in 0 until ws.length()) {
                        val w = ws.optJSONObject(i) ?: continue
                        val lo = w.optDouble("lo_mhz", Double.NaN)
                        val hi = w.optDouble("hi_mhz", Double.NaN)
                        if (lo.isFinite() && hi.isFinite() && hi > lo) add(Wifi(lo, hi, w.optString("label", "Wi-Fi"), w.optBoolean("busy")))
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
