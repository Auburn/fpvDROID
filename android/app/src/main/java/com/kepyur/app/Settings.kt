package com.kepyur.app

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

    /** What the spectrum page shows and sweeps (MHz) for a band: the low band on its own, else all the others. */
    fun span(band: Char): Pair<Int, Int> = if (band == 'L') 5340 to 5640 else 5635 to 5955

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

/**
 * A channel the Spectrum tab marks: an analog one, or one of a digital system's. [wide]: one of a mode that takes
 * 40 MHz or more (DJI V1/V2 at 50 Mbps, the O3's and O4's widest), which DJI numbers apart from the narrow ones.
 */
data class Marker(val name: String, val mhz: Double, val wide: Boolean = false)

/**
 * A set of channels the Spectrum tab can mark over [span] (MHz): an analog band, or a digital system's channel
 * plan. Only an analog one can be watched.
 */
class Plan(val title: String, val markers: List<Marker>, val analog: Boolean, val span: Pair<Int, Int>) {
    private val hasWide = markers.any { it.wide }

    /**
     * The marker a signal at [mhz], [widthMhz] wide, is on (within [Channels.SAME_CHANNEL_MHZ]), or null. In a plan
     * with channels of a wide mode, a signal [Plans.WIDE_MHZ] wide or more is on one of those, a narrower one on
     * one of the others: DJI gives the same frequency another number in another mode.
     */
    fun near(mhz: Double, widthMhz: Double): Marker? = markers.filter {
        abs(it.mhz - mhz) < Channels.SAME_CHANNEL_MHZ && (!hasWide || it.wide == widthMhz >= Plans.WIDE_MHZ)
    }.minByOrNull { abs(it.mhz - mhz) }

    /** A channel of the plan, of any mode, is at [mhz]. */
    fun has(mhz: Double): Boolean = markers.any { abs(it.mhz - mhz) < Channels.SAME_CHANNEL_MHZ }
}

/**
 * What the Spectrum tab can mark: the analog bands, and the channel plans of the digital systems flown beside them
 * (centre frequencies as their makers publish them, FCC). Kepyur shows analog video only: these just say whose
 * channel a signal is on.
 */
object Plans {
    const val ANALOG = 0
    const val DJI = 1
    const val HDZERO = 2
    val systems = listOf("Analog", "DJI", "HDZero")
    /**
     * A signal at least this wide (MHz, as the radio measures it: 6 dB over the floor) is in one of DJI's wide modes:
     * they take 40 or 60 MHz, the narrow ones 10 or 20, analog video 10 to 20.
     */
    const val WIDE_MHZ = 30.0

    private val main = 5635 to 5955

    fun analog(band: Char) = Plan(Channels.bands.first { it.first == band }.second,
        Channels.of(band).map { Marker(it.name, it.mhz.toDouble()) }, analog = true, span = Channels.span(band))

    /** DJI's 5.8 GHz systems, as the picker names them. */
    val djiModels = listOf("V1/V2", "O3", "O4")
    val dji = listOf(
        // The DJI FPV System (goggles V1/V2, Air Unit, Vista): 20 MHz channels at 25 Mbps. At 50 Mbps (FCC only) a
        // channel takes 40 MHz and three are left, CH1-CH3 on the goggles, on the 25 Mbps CH2, CH4 and CH6, and the
        // public CH8 (Oscar Liang's chart and 50 Mbps guide: DJI publishes only the 25 Mbps table). Walksnail
        // Avatar uses the same channels.
        Plan("DJI V1/V2", listOf(5660, 5695, 5735, 5770, 5805, 5878, 5914, 5839).mapIndexed { i, f -> Marker("CH${i + 1}", f.toDouble()) } +
            listOf(Marker("CH1 50M", 5695.0, wide = true), Marker("CH2 50M", 5770.0, wide = true),
                Marker("CH3 50M", 5878.0, wide = true), Marker("CH8 50M", 5839.0, wide = true)), analog = false, span = main),
        // O3 Air Unit: three channels at 20 or 10 MHz, one at 40 (DJI calls it Channel 1 too)
        Plan("DJI O3", listOf(Marker("CH1", 5768.5), Marker("CH2", 5804.5), Marker("CH3", 5839.5), Marker("40M", 5794.5, wide = true)),
            analog = false, span = main),
        // O4 Air Unit: channels 2 and 3 moved from the O3's; one at 40 or 60 MHz
        Plan("DJI O4", listOf(Marker("CH1", 5768.5), Marker("CH2", 5789.5), Marker("CH3", 5814.5), Marker("40/60M", 5794.5, wide = true)),
            analog = false, span = main),
    )

    /** HDZero: Raceband and four more. (Its unlockable L1-L8 are the analog lowband's channels: the L band shows them.) */
    val hdzero = Plan("HDZero", (Channels.of('R') + Channels.of('E')[0] + Channels.of('F')[0] + Channels.of('F')[1] + Channels.of('F')[3])
        .sortedBy { it.mhz }.map { Marker(it.name, it.mhz.toDouble()) }, analog = false, span = main)

    /** The plan the Spectrum tab shows. */
    fun of(s: Settings): Plan = when (s.spectrumSystem) {
        DJI -> dji[s.djiModel.coerceIn(0, dji.size - 1)]
        HDZERO -> hdzero
        else -> analog(s.band)
    }

    /**
     * What to call a signal at [mhz], [widthMhz] wide: by [plan]'s channel, else by an analog channel ([band] first),
     * else by its frequency.
     */
    fun name(mhz: Double, widthMhz: Double, plan: Plan, band: Char): String =
        plan.near(mhz, widthMhz)?.name ?: Channels.nearest(mhz, band, Channels.SAME_CHANNEL_MHZ)?.name ?: "${mhz.roundToInt()}"
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
    var eyeSpacing by pref("eye_spacing", 0f)                // -1..1: pictures apart .. together
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
    var spectrumSystem by pref("spectrum_system", Plans.ANALOG)
    var djiModel by pref("dji_model", 0)                     // Plans.djiModels

    val aspect get() = if (wide) 16f / 9f else 4f / 3f
    /** Fraction of the screen width each eye's picture moves toward the middle. */
    val eyeShift get() = eyeSpacing * EYE_SHIFT_MAX

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

    companion object {
        const val FIT = 0
        const val STRETCH = 1
        const val CROP = 2
        const val SCALE_MIN = 0.5f
        const val SCALE_MAX = 1.5f
        const val EYE_SHIFT_MAX = 0.08f
        const val GAIN_MAX = 45f
    }
}
