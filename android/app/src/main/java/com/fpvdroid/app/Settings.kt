package com.fpvdroid.app

import android.content.Context
import android.content.SharedPreferences
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlin.reflect.KProperty

/** A standard analog 5.8 GHz channel. The table matches native/dsp/spectrum.cpp (fpv_channels). */
data class Channel(val band: Char, val number: Int, val mhz: Int) {
    val name get() = "$band$number"
}

object Channels {
    /** A signal within this of a channel's frequency is on that channel (VTXs are a few MHz out at worst). */
    const val SAME_CHANNEL_MHZ = 4.0

    /** Band letter to long name, in the order the picker shows them (Raceband first: by far the most common). */
    val bands = listOf('R' to "Raceband", 'F' to "Fatshark", 'A' to "Boscam A", 'B' to "Boscam B", 'E' to "Boscam E", 'L' to "Lowband")

    private val table = mapOf(
        'A' to intArrayOf(5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725),
        'B' to intArrayOf(5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866),
        'E' to intArrayOf(5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945),
        'F' to intArrayOf(5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880),
        'R' to intArrayOf(5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917),
        'L' to intArrayOf(5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621),
    )

    fun of(band: Char): List<Channel> = table.getValue(band).mapIndexed { i, mhz -> Channel(band, i + 1, mhz) }

    /** Where [band] is in [bands] (the pickers' order). */
    fun indexOf(band: Char) = bands.indexOfFirst { it.first == band }.coerceAtLeast(0)

    /** The channel a frequency belongs to: within [maxOff] MHz, the nearest; [preferBand] wins a tie (F8 and R7 share 5880). */
    fun nearest(mhz: Double, preferBand: Char? = null, maxOff: Double = 6.0): Channel? {
        var best: Channel? = null
        var bestOff = maxOff
        val order = (listOfNotNull(preferBand) + bands.map { it.first }).distinct()
        for (b in order) for (c in of(b)) {
            val off = abs(c.mhz - mhz)
            if (off < bestOff - 0.01) { bestOff = off; best = c }
        }
        return best
    }
}

/** Everything the menu sets, kept across launches: each is read when this is made, and kept as soon as it is set. */
class Settings(private val prefs: SharedPreferences) {
    constructor(ctx: Context) : this(ctx.getSharedPreferences("settings", Context.MODE_PRIVATE))

    // channel
    var channelMhz by pref("channel_mhz", 0)                 // 0: none picked yet
    var band by pref("band", 'R')
    // view
    var wide by pref("wide", false)                          // false 4:3, true 16:9
    var fit by pref("fit", FIT)
    var scale by pref("scale", 1f)                           // SCALE_MIN..SCALE_MAX
    var goggles by pref("goggles", false)
    // goggles: their own size, in place of scale, and the pictures lined up with their lenses (-1..1, 0 in the middle)
    var gogglesScale by pref("goggles_scale", 1f)            // SCALE_MIN..SCALE_MAX
    var eyeSpacing by pref("eye_spacing", 0f)                // pictures apart .. together: ±EYE_SHIFT_MAX of the width
    var eyeOffsetX by pref("eye_offset_x", 0f)               // both left .. right: ±EYE_OFFSET_X_MAX of the width
    var eyeOffsetY by pref("eye_offset_y", 0f)               // both down .. up: ±EYE_OFFSET_Y_MAX of the height
    // picture
    var lumaComb by pref("luma_comb", true)
    var cleanOsd by pref("clean_osd", true)
    var smoothLines by pref("smooth_lines", true)
    var sparkleRepair by pref("sparkle_repair", false)
    var colourComb by pref("comb", false)                    // the 3-line chroma comb ("comb" is where installs have it)
    // signal
    var rate40 by pref("rate40", true)
    var diversity by pref("diversity", false)
    var offsetTune by pref("offset_tune", true)
    var gainAuto by pref("gain_auto", true)                  // the chip's own gain control
    var gainDb by pref("gain_db", 30f)                       // when it is not: 0..GAIN_MAX
    // spectrum: whose channels it marks (Plans)
    var spectrumSystem by pref("spectrum_marks", ChannelSystem.ANALOG)
    var djiModel by pref("dji_model", 0)                     // Plans.djiModels

    /** The channel picked is [ch]; on none, the frequency [mhz] itself (a transmitter off every channel). */
    fun choose(ch: Channel?, mhz: Double) {
        channelMhz = ch?.mhz ?: mhz.roundToInt()
        if (ch != null) band = ch.band
    }

    val aspect get() = if (wide) 16f / 9f else 4f / 3f
    /** How the picture is laid out ([Engine.setView]): with goggles, at their own size and lined up with their lenses. */
    val view: ViewOpts get() {
        if (!goggles) return ViewOpts(aspect, fit, scale)
        val lenses = Goggles(eyeSpacing * EYE_SHIFT_MAX, eyeOffsetX * EYE_OFFSET_X_MAX, eyeOffsetY * EYE_OFFSET_Y_MAX)
        return ViewOpts(aspect, fit, gogglesScale, lenses)
    }

    /** One setting: its value, and how it is written back when it is set. */
    private inner class Pref<T>(private var value: T, private val put: SharedPreferences.Editor.(T) -> Unit) {
        operator fun getValue(owner: Settings, property: KProperty<*>): T = value
        operator fun setValue(owner: Settings, property: KProperty<*>, to: T) {
            value = to
            prefs.edit().apply { put(to) }.apply()
        }
    }

    private fun pref(key: String, default: Boolean) = Pref(prefs.getBoolean(key, default)) { putBoolean(key, it) }
    private fun pref(key: String, default: Int) = Pref(prefs.getInt(key, default)) { putInt(key, it) }
    private fun pref(key: String, default: Float) = Pref(prefs.getFloat(key, default)) { putFloat(key, it) }
    /** A letter, kept as a string. */
    private fun pref(key: String, default: Char) =
        Pref(prefs.getString(key, default.toString())!!.first()) { putString(key, it.toString()) }
    /** One of an enum's values, kept by its name (a name it no longer has: the default). */
    private inline fun <reified E : Enum<E>> pref(key: String, default: E) =
        Pref(enumValues<E>().firstOrNull { it.name == prefs.getString(key, null) } ?: default) { putString(key, it.name) }

    companion object {
        const val FIT = 0
        const val STRETCH = 1
        const val CROP = 2
        const val SCALE_MIN = 0.5f
        const val SCALE_MAX = 1.5f
        const val EYE_SHIFT_MAX = 0.08f
        const val EYE_OFFSET_X_MAX = 0.1f
        const val EYE_OFFSET_Y_MAX = 0.2f
        const val GAIN_MAX = 45f
    }
}
