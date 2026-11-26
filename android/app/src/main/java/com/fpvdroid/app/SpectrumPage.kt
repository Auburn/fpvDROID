package com.fpvdroid.app

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.os.SystemClock
import android.view.Gravity
import android.view.View
import android.widget.HorizontalScrollView
import android.widget.LinearLayout
import kotlin.math.roundToInt

/**
 * The menu's Spectrum tab, in [page]: whose channels are marked along the top (an analog band, a DJI model or
 * HDZero), the graph ([SpectrumView]) under it, and at the foot Sweep and a chip for each thing on the air and each
 * Wi-Fi network. Like the
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
    private var wifiShown: List<Signal>? = null
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
     * the digital system the radio judged it to be, if any: [Spot.name]); then the Wi-Fi networks.
     */
    private fun showChips() {
        val chips = graph.onAir.map { it.spot.name to barsFor(it.carrier.snrDb).coerceAtLeast(1) }
        val wifi = graph.wifi
        if (chips == chipsShown && wifi == wifiShown) return
        chipsShown = chips
        wifiShown = wifi
        chipRow.removeAllViews()
        for ((name, bars) in chips) {
            val chip = FoundChip(ctx, name, bars)
            // what is on the air under that name when the chip is tapped (the chips stay while only the numbers move)
            chip.setOnClickListener { graph.onAir.firstOrNull { it.spot.name == name }?.let { pick(it.spot) } }
            chipRow.addView(chip, LinearLayout.LayoutParams(-2, -2).apply { marginEnd = dpi(8f) })
        }
        for (n in wifi) {
            val chip = WifiChip(ctx, n.label, n.active)
            // there is no picture to be had from a network either
            chip.setOnClickListener {
                analogOnlyAt = SystemClock.uptimeMillis()
                render(state)
            }
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
            !sp?.carriers.isNullOrEmpty() || graph.wifi.isNotEmpty() -> "In use"
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

/**
 * A Wi-Fi network under the Spectrum tab's graph: its name, and in plain words whether it has been seen in use or only
 * heard sending its beacons ("idle"). A chip like [FoundChip], edged in the graph's Wi-Fi colour, with no bars: how
 * strong a network is says less than whether it is in use.
 */
internal class WifiChip(ctx: Context, private val title: String, busy: Boolean) : View(ctx) {
    private val words = if (busy) "in use" else "idle"
    private val edge = Color.argb(if (busy) 150 else 80, Color.red(Skin.WARN), Color.green(Skin.WARN), Color.blue(Skin.WARN))
    private val paint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { typeface = Skin.MEDIUM; textSize = sp(13.5f); color = Skin.TEXT }
    private val small = Paint(Paint.ANTI_ALIAS_FLAG).apply { typeface = Skin.REGULAR; textSize = sp(12.5f); color = Skin.TEXT_3 }
    private val box = RectF()

    init { isClickable = true; contentDescription = "$title, $words" }

    override fun onMeasure(w: Int, h: Int) =
        setMeasuredDimension((text.measureText(title) + small.measureText(words) + dp(36f)).roundToInt(), dpi(36f))

    override fun onDraw(c: Canvas) {
        val h = height.toFloat()
        box.set(dp(0.75f), dp(0.75f), width - dp(0.75f), h - dp(0.75f))
        paint.style = Paint.Style.FILL
        paint.color = if (isPressed) Skin.FILL_STRONG else Skin.FILL
        c.drawRoundRect(box, h / 2, h / 2, paint)
        paint.style = Paint.Style.STROKE
        paint.strokeWidth = dp(1f)
        paint.color = edge
        c.drawRoundRect(box, h / 2, h / 2, paint)
        paint.style = Paint.Style.FILL
        val base = h / 2 - (text.descent() + text.ascent()) / 2
        c.drawText(title, dp(14f), base, text)
        c.drawText(words, dp(14f) + text.measureText(title) + dp(8f), base, small)
    }

    override fun drawableStateChanged() {
        super.drawableStateChanged()
        invalidate()
    }
}
