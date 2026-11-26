package com.fpvdroid.app

import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * A channel the Spectrum tab marks: an analog one ([channel], which tuning it watches), or one of a digital system's
 * (no [channel]: there is no picture to be had from it). [wide]: one of a mode that takes 40 MHz or more (DJI V1/V2
 * at 50 Mbps, the O3's and O4's widest), which DJI numbers apart from the narrow ones.
 */
data class Marker(val name: String, val mhz: Double, val wide: Boolean = false, val channel: Channel? = null)

/**
 * Where the Spectrum tab puts a frequency: on [plan][Plan]'s channel [marker], and on the analog [channel] it would
 * tune. Its [name] is what it is shown as, and what picking it tunes is that same channel (or the frequency
 * itself, on none), unless it is [digital]. [kind]: the digital system the radio judged the signal there to be, said
 * before the channel's name ("DJI CH3", "HDZero R4").
 */
data class Spot(val mhz: Double, val marker: Marker?, val channel: Channel?, val kind: SignalKind? = null) {
    val name: String get() = (marker?.name ?: channel?.name ?: "${mhz.roundToInt()}").let { if (kind != null) "${kind.label} $it" else it }
    /** On a digital system's channel, or a signal the radio judged digital: only analog video can be watched. */
    val digital: Boolean get() = kind != null || (marker != null && channel == null)
}

/** Whose channels the Spectrum tab marks: an analog band's, DJI's (one of its models) or HDZero's. */
enum class ChannelSystem(val label: String) { ANALOG("Analog"), DJI("DJI"), HDZERO("HDZero") }

/**
 * A set of channels the Spectrum tab can mark over [span] (MHz): an analog band, or a digital system's channel
 * plan. Only an analog one can be watched.
 */
data class Plan(val title: String, val markers: List<Marker>, val analog: Boolean, val span: Pair<Int, Int>) {
    private val hasWide = markers.any { it.wide }

    /**
     * The marker a signal at [mhz], [widthMhz] wide, is on (within [Channels.SAME_CHANNEL_MHZ]), or null. In a plan
     * with channels of a wide mode, a signal [Plans.WIDE_MHZ] wide or more is on one of those, a narrower one on
     * one of the others: DJI gives the same frequency another number in another mode.
     */
    fun near(mhz: Double, widthMhz: Double): Marker? = markers.filter {
        abs(it.mhz - mhz) < Channels.SAME_CHANNEL_MHZ && (!hasWide || it.wide == widthMhz >= Plans.WIDE_MHZ)
    }.minByOrNull { abs(it.mhz - mhz) }

    /**
     * A signal at [mhz], [widthMhz] wide: on this plan's channel, else on the analog channel it is on ([band] first on
     * a tie). [kind]: what the radio judged it to be, if anything.
     */
    fun spot(mhz: Double, widthMhz: Double, band: Char, kind: SignalKind? = null): Spot {
        val m = near(mhz, widthMhz)
        return Spot(mhz, m, if (m != null) m.channel else Channels.nearest(mhz, band, Channels.SAME_CHANNEL_MHZ), kind)
    }

    /** One of this plan's channels itself. */
    fun spot(m: Marker) = Spot(m.mhz, m, m.channel)
}

/**
 * What the Spectrum tab can mark: the analog bands, and the channel plans of the digital systems flown beside them
 * (centre frequencies as their makers publish them, FCC). fpvDROID shows analog video only: these just say whose
 * channel a signal is on.
 */
object Plans {
    /**
     * A signal at least this wide (MHz, as the radio measures it: 6 dB over the floor) is in one of DJI's wide modes:
     * they take 40 or 60 MHz, the narrow ones 10 or 20, analog video 10 to 20.
     */
    const val WIDE_MHZ = 30.0

    /** What the tab shows and sweeps (MHz): the low band on its own, every other band and system together. */
    val LOW_SPAN = 5340 to 5640
    val MAIN_SPAN = 5635 to 5955

    fun analog(band: Char) = Plan(Channels.bands.first { it.first == band }.second,
        Channels.of(band).map { Marker(it.name, it.mhz.toDouble(), channel = it) }, analog = true,
        span = if (band == 'L') LOW_SPAN else MAIN_SPAN)

    private fun digital(title: String, markers: List<Marker>) = Plan(title, markers, analog = false, span = MAIN_SPAN)

    /** DJI's 5.8 GHz systems, by the name the picker gives each. */
    private val djiByModel = linkedMapOf(
        // The DJI FPV System (goggles V1/V2, Air Unit, Vista): 20 MHz channels at 25 Mbps. At 50 Mbps (FCC only) a
        // channel takes 40 MHz and three are left, CH1-CH3 on the goggles, on the 25 Mbps CH2, CH4 and CH6, and the
        // public CH8 (Oscar Liang's chart and 50 Mbps guide: DJI publishes only the 25 Mbps table). Walksnail
        // Avatar uses the same channels.
        "V1/V2" to digital("DJI V1/V2",
            listOf(5660, 5695, 5735, 5770, 5805, 5878, 5914, 5839).mapIndexed { i, f -> Marker("CH${i + 1}", f.toDouble()) } +
                listOf(Marker("CH1 50M", 5695.0, wide = true), Marker("CH2 50M", 5770.0, wide = true),
                    Marker("CH3 50M", 5878.0, wide = true), Marker("CH8 50M", 5839.0, wide = true))),
        // O3 Air Unit: three channels at 20 or 10 MHz, one at 40 (DJI calls it Channel 1 too)
        "O3" to digital("DJI O3",
            listOf(Marker("CH1", 5768.5), Marker("CH2", 5804.5), Marker("CH3", 5839.5), Marker("40M", 5794.5, wide = true))),
        // O4 Air Unit: channels 2 and 3 moved from the O3's; one at 40 or 60 MHz
        "O4" to digital("DJI O4",
            listOf(Marker("CH1", 5768.5), Marker("CH2", 5789.5), Marker("CH3", 5814.5), Marker("40/60M", 5794.5, wide = true))),
    )
    val djiModels = djiByModel.keys.toList()
    val dji = djiByModel.values.toList()

    /** HDZero: Raceband and four more. (Its unlockable L1-L8 are the analog lowband's channels: the L band shows them.) */
    val hdzero = digital("HDZero", (Channels.of('R') + Channels.of('E')[0] + Channels.of('F')[0] + Channels.of('F')[1] +
        Channels.of('F')[3]).sortedBy { it.mhz }.map { Marker(it.name, it.mhz.toDouble()) })

    /** The plan the Spectrum tab shows. */
    fun of(s: Settings): Plan = when (s.spectrumSystem) {
        ChannelSystem.ANALOG -> analog(s.band)
        ChannelSystem.DJI -> dji[s.djiModel.coerceIn(dji.indices)]
        ChannelSystem.HDZERO -> hdzero
    }
}
