package com.fpvdroid.app

import android.os.SystemClock
import android.view.Gravity
import android.view.View
import android.widget.HorizontalScrollView
import android.widget.LinearLayout

/**
 * The menu's Spectrum tab, in [page]: whose channels are marked along the top (an analog band, a DJI model or
 * HDZero), the graph ([SpectrumView]) under it, and at the foot Sweep and a chip for each thing on the air. Like the
 * rest of the menu it changes [Settings] itself and tells the [MenuView.Listener] what it did; it knows nothing of
 * the radio. [syncChannel]: the menu's own, for a change the Channel tab shows too (the band, the channel).
 */
internal class SpectrumPage(
    private val page: LinearLayout,
    private val s: Settings,
    private val on: MenuView.Listener,
    private val syncChannel: () -> Unit,
) {
    private val ctx = page.context
    private val system: Segmented
    private val band: Segmented
    private val dji: Segmented
    private val graph = SpectrumView(ctx).apply { onPick = { pick(it) } }
    private val sweepButton = PillButton(ctx, "Sweep").apply { setOnClickListener { on.onSweep(true) } }
    private val sweepText = label(ctx, "", 12.5f, Skin.TEXT_2)
    private val chipRow = LinearLayout(ctx).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
    private var chipsShown: List<Pair<String, Int>>? = null   // name, bars
    private var analogOnlyAt = -ANALOG_ONLY_MS                // when a digital channel was last picked (uptimeMillis)
    private var state = MenuState()

    init {
        val head = LinearLayout(ctx).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        system = Segmented(ctx, ChannelSystem.entries.map { it.label }, s.spectrumSystem.ordinal) { i ->
            replan { s.spectrumSystem = ChannelSystem.entries[i] }
            sync()
        }.apply { label = "Channels" }
        head.addView(system, LinearLayout.LayoutParams(dpi(216f), dpi(34f)))
        head.addView(View(ctx), LinearLayout.LayoutParams(0, 1, 1f))
        // the same band as the Channel tab's: which channels are marked, and (the low band or not) which span is swept
        band = Segmented(ctx, Channels.bands.map { it.first.toString() }, Channels.indexOf(s.band)) { i ->
            replan { s.band = Channels.bands[i].first }
            syncChannel()
        }.apply { label = "Band" }
        head.addView(band, LinearLayout.LayoutParams(dpi(228f), dpi(34f)))
        dji = Segmented(ctx, Plans.djiModels, s.djiModel) { i ->
            replan { s.djiModel = i }
            sync()
        }.apply { label = "DJI" }
        head.addView(dji, LinearLayout.LayoutParams(dpi(186f), dpi(34f)))
        page.addView(head)
        // the graph takes whatever height the card has left (the page fills the card: MenuView.scrolling)
        page.addView(graph, LinearLayout.LayoutParams(-1, 0, 1f).apply { topMargin = dpi(10f) })
        val foot = LinearLayout(ctx).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        foot.addView(sweepButton, LinearLayout.LayoutParams(-2, dpi(40f)))
        foot.addView(sweepText, LinearLayout.LayoutParams(-2, -2).apply { marginStart = dpi(14f) })
        val chips = HorizontalScrollView(ctx).apply {
            isHorizontalScrollBarEnabled = false
            overScrollMode = View.OVER_SCROLL_NEVER
            addView(chipRow)
        }
        foot.addView(chips, LinearLayout.LayoutParams(0, -2, 1f).apply { marginStart = dpi(12f) })
        page.addView(foot, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dpi(12f) })
    }

    private fun dpi(v: Float) = page.dpi(v)

    /** The tab follows [Settings]: whose channels it marks, the band, the channel being watched. */
    fun sync() {
        val plan = Plans.of(s)
        system.select(s.spectrumSystem.ordinal)
        band.select(Channels.indexOf(s.band))
        dji.select(s.djiModel)
        band.visibility = if (s.spectrumSystem == ChannelSystem.ANALOG) View.VISIBLE else View.GONE
        dji.visibility = if (s.spectrumSystem == ChannelSystem.DJI) View.VISIBLE else View.GONE
        graph.plan = plan
        graph.band = s.band
        graph.chosenMhz = if (plan.analog) s.channelMhz.toDouble() else 0.0
        showChips()
    }

    /** Whose channels are marked changes ([change] keeps it in [Settings]); the listener hears if the span moved. */
    private fun replan(change: () -> Unit) {
        val was = Plans.of(s).span
        change()
        if (Plans.of(s).span != was) on.onSpectrumSpan()
    }

    /** Something picked on the tab (a carrier, a channel's name, a chip): tuned as it is named, unless it is digital. */
    private fun pick(spot: Spot) {
        if (spot.digital) {
            analogOnlyAt = SystemClock.uptimeMillis()
            render(state)
            return
        }
        s.choose(spot.channel, spot.mhz)
        syncChannel()
        on.onTune()
    }

    /**
     * The chips under the graph: what is on the air, left to right as in the graph, each named where it puts it (after
     * the digital system the radio judged it to be, if any: [Spot.name]).
     */
    private fun showChips() {
        val chips = graph.onAir.map { it.spot.name to barsFor(it.carrier.snrDb).coerceAtLeast(1) }
        if (chips == chipsShown) return
        chipsShown = chips
        chipRow.removeAllViews()
        for ((name, bars) in chips) {
            val chip = FoundChip(ctx, name, bars)
            // what is on the air under that name when the chip is tapped (the chips stay while only the numbers move)
            chip.setOnClickListener { graph.onAir.firstOrNull { it.spot.name == name }?.let { pick(it.spot) } }
            chipRow.addView(chip, LinearLayout.LayoutParams(-2, -2).apply { marginEnd = dpi(8f) })
        }
    }

    fun render(st: MenuState) {
        state = st
        val sp = st.spectrum
        graph.spectrum = sp
        graph.emptyText = when {
            !st.radioReady -> "The spectrum shows once the radio is running"
            st.sweeping || st.scanning -> "Sweeping…"
            else -> "Press Sweep to look at the band"
        }
        sweepButton.isEnabled = st.radioReady && !st.sweeping && !st.scanning
        sweepButton.text = if (st.sweeping) "Sweeping" else "Sweep"
        val (lo, hi) = graph.plan.span
        val words = when {
            SystemClock.uptimeMillis() - analogOnlyAt < ANALOG_ONLY_MS -> "Only analog video can be shown"
            !st.radioReady -> ""
            !sp?.carriers.isNullOrEmpty() -> "In use"
            st.scanning -> "Waiting for the scan"
            st.sweeping && (sp == null || sp.sweeps == 0) -> "$lo–$hi MHz"
            sp != null && !sp.empty -> "No signals"
            else -> ""
        }
        if (sweepText.text.toString() != words) sweepText.text = words
        sweepText.visibility = if (words.isEmpty()) View.GONE else View.VISIBLE
        showChips()
    }

    companion object {
        /** How long the tab says that a digital channel cannot be watched. */
        const val ANALOG_ONLY_MS = 3000L
    }
}
