package com.fpvdroid.app

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.graphics.Shader
import android.view.MotionEvent
import android.view.View
import android.view.ViewConfiguration
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * The spectrum page's graph: the spectrum over [plan]'s span, its channels marked under it (in use: lit; the chosen
 * one: white), the stretch being swept now shaded, and each Wi-Fi network seen a span of its own, named at the top. A
 * tap picks for [onPick]: the carrier nearest the finger, or else the channel nearest it. [band]: the analog band a
 * signal off [plan]'s channels is put on.
 */
class SpectrumView(ctx: Context) : View(ctx) {
    var spectrum: Spectrum? = null
        set(v) { field = v; place() }
    var plan: Plan = Plans.analog('R')
        set(v) { if (field != v) { field = v; place() } }
    var band = 'R'
        set(v) { if (field != v) { field = v; place() } }
    /** What is on the air, left to right, each where [plan] puts it: what the graph lights, and the tab lists. */
    var onAir: List<OnAir> = emptyList()
        private set
    /** The Wi-Fi networks seen in the span, left to right. */
    var wifi: List<Wifi> = emptyList()
        private set
    /** The channel being watched, if it is one of [plan]'s (MHz; 0: none). */
    var chosenMhz = 0.0
        set(v) { if (field != v) { field = v; names = null; invalidate() } }
    private val span get() = plan.span
    /** Said in the middle of the graph while there is nothing to draw. */
    var emptyText = ""
        set(v) { if (field != v) { field = v; invalidate() } }
    var onPick: ((Spot) -> Unit)? = null

    private val paint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val line = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = dp(1.6f)
        strokeJoin = Paint.Join.ROUND
        color = Skin.ACCENT
    }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG)
    private val small = Paint(Paint.ANTI_ALIAS_FLAG).apply { typeface = Skin.REGULAR; textSize = sp(10.5f); color = Skin.TEXT_3 }
    private val name = Paint(Paint.ANTI_ALIAS_FLAG).apply { typeface = Skin.MEDIUM; textSize = sp(12.5f); textAlign = Paint.Align.CENTER }
    private val middle = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        typeface = Skin.REGULAR; textSize = sp(13f); color = Skin.TEXT_2; textAlign = Paint.Align.CENTER
    }
    private val wifiText = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        typeface = Skin.MEDIUM; textSize = sp(11f); color = Skin.WARN; textAlign = Paint.Align.CENTER
    }
    private val wifiEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = dp(1f) }
    private val dashes = DashPathEffect(floatArrayOf(dp(4f), dp(4f)), 0f)
    private val box = RectF()
    private val trace = Path()
    private val area = Path()
    private val clip = Path()
    private var shaderTop = Float.NaN
    private var shaderBottom = Float.NaN
    private var topDb = 30f
    /** The plan's channels in the span, each with the row of names it is in (see [names]); null: to be worked out. */
    private var names: List<Pair<Marker, Int>>? = null

    init {
        isClickable = true
        isFocusable = true
        describe()
    }

    /** The spectrum, the plan or the band changed: what is on the air is put on the plan's channels again. */
    private fun place() {
        onAir = spectrum?.carriers.orEmpty().sortedBy { it.mhz }.map { OnAir(it, plan.spot(it.mhz, it.widthMhz, band)) }
        wifi = spectrum?.wifi.orEmpty().filter { it.hiMhz > span.first && it.loMhz < span.second }.sortedBy { it.loMhz }
        names = null
        describe()
        invalidate()
    }

    override fun onMeasure(w: Int, h: Int) = setMeasuredDimension(resolveSize(dpi(400f), w), resolveSize(dpi(150f), h))

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        names = null
    }

    // ------------------------------------------------------------------------------------------ geometry
    /** Something on the air is on [m]: the channel is in use. */
    private fun lit(m: Marker): Boolean = onAir.any { it.spot.marker == m }

    /**
     * The channels shown: all of the plan's in the span, but where a wide mode's channel has a narrow one's
     * frequency (DJI V1/V2's 50 Mbps channels are on its 25 Mbps CH2, CH4, CH6, CH8) only one of the two, the
     * wide one while it is in use.
     */
    private fun shown(): List<Marker> {
        val all = plan.markers.filter { it.mhz >= span.first && it.mhz <= span.second }
        return all.filter { m ->
            val twin = all.firstOrNull { it.wide != m.wide && abs(it.mhz - m.mhz) < 1.0 }
            when {
                twin == null -> true
                m.wide -> lit(m)
                else -> !lit(twin) || lit(m)
            }
        }
    }

    /** The names under the plot, for a test. */
    internal fun shownNames(): List<String> = names().map { it.first.name }

    /**
     * Where each channel's name goes under the plot: the first row, unless it would run into a name already there
     * (channels 5 to 10 MHz apart: HDZero's, DJI O4's), then the second.
     */
    private fun names(): List<Pair<Marker, Int>> = names ?: run {
        val gap = dp(6f)
        val ends = floatArrayOf(-1e9f, -1e9f)   // where the last name in each row ends
        shown().sortedBy { it.mhz }.map { m ->
            val half = name.measureText(m.name) / 2 + if (m.mhz == chosenMhz) dp(7f) else 0f
            val x = xOf(m.mhz)
            val row = when {
                x - half >= ends[0] + gap -> 0
                x - half >= ends[1] + gap -> 1
                ends[0] <= ends[1] -> 0
                else -> 1
            }
            ends[row] = x + half
            m to row
        }.also { names = it }
    }

    /** Room under the plot for the channel names (one or two rows) and the frequencies. */
    private val under get() = dp(if (names().any { it.second == 1 }) 56f else 40f)
    private val plotBottom get() = height - under
    private val plotTop get() = dp(1f)

    private fun xOf(mhz: Double): Float {
        val (lo, hi) = span
        return ((mhz - lo) / (hi - lo) * width).toFloat()
    }

    private fun mhzAt(x: Float): Double {
        val (lo, hi) = span
        return lo + x / width.coerceAtLeast(1) * (hi - lo).toDouble()
    }

    private fun yOf(db: Float): Float {
        val top = plotTop + dp(10f)
        val t = ((db - BOTTOM_DB) / (topDb - BOTTOM_DB)).coerceIn(0f, 1.04f)
        return plotBottom - t * (plotBottom - top)
    }

    // ------------------------------------------------------------------------------------------ drawing
    override fun onDraw(c: Canvas) {
        val w = width.toFloat()
        val bottom = plotBottom
        val r = dp(14f)
        val sp = spectrum
        // the scale: at least 30 dB, else the strongest carrier and a little room, in steps of 10
        val peak = sp?.carriers?.maxOfOrNull { it.snrDb } ?: 0f
        topDb = (ceil((peak + 4f) / 10f) * 10f).coerceIn(30f, 60f)

        box.set(dp(0.75f), plotTop, w - dp(0.75f), bottom)
        paint.style = Paint.Style.FILL
        paint.color = Skin.FILL
        c.drawRoundRect(box, r, r, paint)
        c.save()
        clip.reset()
        clip.addRoundRect(box, r, r, Path.Direction.CW)
        c.clipPath(clip)

        // the stretch being swept now
        if (sp != null && sp.atMhz > 0) {
            val x0 = xOf(sp.atMhz - sp.atWidthMhz / 2)
            val x1 = xOf(sp.atMhz + sp.atWidthMhz / 2)
            if (x1 > 0 && x0 < w) {
                paint.color = Color.argb(16, 255, 255, 255)
                c.drawRect(max(0f, x0), plotTop, min(w, x1), bottom, paint)
            }
        }
        drawWifiSpans(c, bottom)
        // dB lines, labelled at the right
        paint.color = Color.argb(14, 255, 255, 255)
        small.textAlign = Paint.Align.RIGHT
        var db = 10f
        while (db < topDb) {
            val y = yOf(db)
            c.drawRect(0f, y, w, y + max(1f, dp(0.5f)), paint)
            c.drawText("${db.roundToInt()} dB", w - dp(10f), y - dp(4f), small)
            db += 10f
        }
        // the channels: a faint line each, brighter when in use or chosen
        for ((m, _) in names()) {
            val x = xOf(m.mhz)
            paint.color = when {
                m.mhz == chosenMhz -> Color.argb(110, 255, 255, 255)
                lit(m) -> Color.argb(90, Color.red(Skin.ACCENT), Color.green(Skin.ACCENT), Color.blue(Skin.ACCENT))
                else -> Color.argb(20, 255, 255, 255)
            }
            c.drawRect(x - dp(0.5f), plotTop, x + dp(0.5f), bottom, paint)
        }
        // the spectrum: a filled line, broken where nothing has been seen
        if (sp != null && !sp.empty) drawTrace(c, sp, bottom)
        else if (emptyText.isNotEmpty()) c.drawText(emptyText, w / 2, (plotTop + bottom) / 2 - (middle.descent() + middle.ascent()) / 2, middle)
        drawWifiNames(c)
        c.restore()
        paint.style = Paint.Style.STROKE
        paint.strokeWidth = dp(1f)
        paint.color = Skin.STROKE
        c.drawRoundRect(box, r, r, paint)
        paint.style = Paint.Style.FILL

        drawChannelNames(c, bottom)
        // frequencies every 50 MHz, under the names
        small.textAlign = Paint.Align.CENTER
        var f = ((span.first + 49) / 50) * 50
        while (f <= span.second) {
            val x = xOf(f.toDouble())
            val half = small.measureText("$f") / 2
            if (x - half >= 0 && x + half <= w) c.drawText("$f", x, height - dp(3f), small)
            f += 50
        }
    }

    private fun drawTrace(c: Canvas, sp: Spectrum, bottom: Float) {
        trace.reset()
        area.reset()
        var open = false
        var startX = 0f
        var lastX = 0f
        for (i in sp.db.indices) {
            val v = sp.db[i]
            val x = xOf(sp.firstMhz + i * sp.binMhz)
            if (v.isNaN()) {
                if (open) { area.lineTo(lastX, bottom); area.lineTo(startX, bottom); area.close() }
                open = false
                continue
            }
            val y = yOf(v)
            if (!open) {
                trace.moveTo(x, y)
                area.moveTo(x, y)
                startX = x
                open = true
            } else {
                trace.lineTo(x, y)
                area.lineTo(x, y)
            }
            lastX = x
        }
        if (open) { area.lineTo(lastX, bottom); area.lineTo(startX, bottom); area.close() }
        val top = yOf(topDb)
        if (top != shaderTop || bottom != shaderBottom) {
            shaderTop = top
            shaderBottom = bottom
            val a = Skin.ACCENT
            fill.shader = LinearGradient(0f, top, 0f, bottom,
                Color.argb(120, Color.red(a), Color.green(a), Color.blue(a)), Color.argb(10, Color.red(a), Color.green(a), Color.blue(a)),
                Shader.TileMode.CLAMP)
        }
        c.drawPath(area, fill)
        c.drawPath(trace, line)
    }

    /**
     * Each Wi-Fi network over the whole height of the plot, behind the spectrum: tinted, with solid edges when it has
     * been seen in use, fainter with dashed ones when only its beacons have been heard (how wide it goes when it is
     * used is not known yet).
     */
    private fun drawWifiSpans(c: Canvas, bottom: Float) {
        val r = Color.red(Skin.WARN)
        val g = Color.green(Skin.WARN)
        val b = Color.blue(Skin.WARN)
        for (n in wifi) {
            val x0 = max(0f, xOf(n.loMhz))
            val x1 = min(width.toFloat(), xOf(n.hiMhz))
            paint.color = Color.argb(if (n.busy) 30 else 14, r, g, b)
            c.drawRect(x0, plotTop, x1, bottom, paint)
            wifiEdge.color = Color.argb(if (n.busy) 130 else 90, r, g, b)
            wifiEdge.pathEffect = if (n.busy) null else dashes
            for (x in floatArrayOf(x0, x1)) c.drawLine(x, plotTop, x, bottom, wifiEdge)
        }
    }

    /** Each network's name in a tag at the top of its span, over the spectrum: a second row for one that would run into the first. */
    private fun drawWifiNames(c: Canvas) {
        val gap = dp(6f)
        var end = -1e9f   // where the last tag in the first row ends
        for (n in wifi) {
            val half = wifiText.measureText(n.label) / 2 + dp(7f)
            val x = ((xOf(max(n.loMhz, span.first.toDouble())) + xOf(min(n.hiMhz, span.second.toDouble()))) / 2)
                .coerceIn(half + dp(3f), max(half + dp(3f), width - half - dp(3f)))
            val row = if (x - half >= end + gap) 0 else 1
            if (row == 0) end = x + half
            val top = plotTop + dp(6f) + row * dp(20f)
            box.set(x - half, top, x + half, top + dp(17f))
            paint.color = Color.argb(215, Color.red(Skin.PANEL), Color.green(Skin.PANEL), Color.blue(Skin.PANEL))
            c.drawRoundRect(box, box.height() / 2, box.height() / 2, paint)
            c.drawText(n.label, x, box.centerY() - (wifiText.descent() + wifiText.ascent()) / 2, wifiText)
        }
    }

    private fun drawChannelNames(c: Canvas, bottom: Float) {
        for ((m, row) in names()) {
            val base = bottom + dp(17f) + row * dp(16f)
            val x = xOf(m.mhz)
            val used = lit(m)
            val chosen = m.mhz == chosenMhz
            if (chosen) {
                val half = name.measureText(m.name) / 2 + dp(7f)
                box.set(x - half, base - dp(13f), x + half, base + dp(4.5f))
                paint.color = Color.argb(235, 255, 255, 255)
                c.drawRoundRect(box, box.height() / 2, box.height() / 2, paint)
            }
            name.color = if (chosen) Skin.ON_LIGHT else if (used) Skin.ACCENT else Skin.TEXT_3
            name.typeface = if (used || chosen) Skin.MEDIUM else Skin.REGULAR
            val half = name.measureText(m.name) / 2
            c.drawText(m.name, x.coerceIn(half + dp(2f), width - half - dp(2f)), base, name)
        }
    }

    // ------------------------------------------------------------------------------------------ touch
    private var downX = 0f
    private var downY = 0f

    @SuppressLint("ClickableViewAccessibility")   // performClick is called, after the pick
    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (!isEnabled) return false
        val slop = ViewConfiguration.get(context).scaledTouchSlop
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> { downX = e.x; downY = e.y }
            MotionEvent.ACTION_UP ->
                if (abs(e.x - downX) <= slop && abs(e.y - downY) <= slop) {
                    pick(e.x, e.y)
                    performClick()
                }
        }
        return true
    }

    /** What a tap at ([x], [y]) means: on the names, the channel under the finger; on the plot, the nearest carrier first. */
    internal fun pick(x: Float, y: Float) {
        val mhz = mhzAt(x)
        val carrier = if (y >= plotBottom) null
        else onAir.filter { abs(it.carrier.mhz - mhz) <= PICK_CARRIER_MHZ }.minByOrNull { abs(it.carrier.mhz - mhz) }
        val to = carrier?.spot ?: names().minByOrNull { abs(it.first.mhz - mhz) }?.let { plan.spot(it.first) } ?: return
        onPick?.invoke(to)
    }

    /** The graph in words, for a screen reader: the span, whose channels, what is in use, and the Wi-Fi seen. */
    private fun describe() {
        val sp = spectrum
        val used = onAir.map { it.spot.name }
        contentDescription = "Spectrum, ${span.first} to ${span.second} megahertz, ${plan.title} channels. " +
            (if (sp == null || sp.empty) "Nothing measured yet" else if (used.isEmpty()) "No signals" else "In use: ${used.joinToString()}") +
            if (wifi.isEmpty()) "" else ". Wi-Fi: ${wifi.joinToString { it.label + if (it.busy) ", in use" else ", idle" }}"
    }

    companion object {
        const val BOTTOM_DB = -5f
        /** A tap this close to a carrier picks it. */
        const val PICK_CARRIER_MHZ = 10.0
    }
}
