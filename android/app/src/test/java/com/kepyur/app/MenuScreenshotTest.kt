package com.kepyur.app

import android.app.Activity
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.RectF
import android.graphics.Shader
import android.os.Looper
import android.view.View
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.Robolectric
import org.robolectric.RobolectricTestRunner
import org.robolectric.Shadows.shadowOf
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode
import java.io.File

/**
 * Renders [MenuView] off screen to PNGs, for looking at the menu without a device.
 * The menu is laid over black, as the screen behind it is, with a stand-in for the picture where the preview is
 * (on the phone the native screen draws the real one there, through the hole the preview clears in the window).
 *
 *   gradle testDebugUnitTest --tests com.kepyur.app.MenuScreenshotTest -Pshots=/some/dir
 *
 * Without -Pshots the PNGs go to app/build/menu-shots.
 */
@RunWith(RobolectricTestRunner::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [35])
class MenuScreenshotTest {
    private val outDir: File by lazy {
        File(System.getProperty("kepyur.shots") ?: "build/menu-shots").also { it.mkdirs() }
    }

    /** Hears where the preview is, and whether a slider has the picture full size, and nothing else. */
    private class Quiet : MenuView.Listener {
        val preview = RectF()
        var radius = 0f
        var peeking = false
        override fun onTune() {}
        override fun onScan() {}
        override fun onView() {}
        override fun onPicture() {}
        override fun onSignal() {}
        override fun onPeek(on: Boolean) { peeking = on }
        override fun onStatusAction() {}
        override fun onClose() {}
        override fun onPreview(left: Int, top: Int, right: Int, bottom: Int, radius: Float) {
            preview.set(left.toFloat(), top.toFloat(), right.toFloat(), bottom.toFloat())
            this.radius = radius
        }
    }

    /**
     * A sky over a field where the picture is, so that it can be told from the panels: in the preview, or the whole
     * screen while a slider is held (a peek).
     */
    private fun standIn(c: Canvas, at: Quiet) {
        val where = if (at.peeking) RectF(0f, 0f, c.width.toFloat(), c.height.toFloat()) else at.preview
        if (where.isEmpty) return
        val p = Paint(Paint.ANTI_ALIAS_FLAG)
        p.shader = LinearGradient(0f, where.top, 0f, where.bottom, Color.rgb(70, 140, 200), Color.rgb(150, 110, 60),
            Shader.TileMode.CLAMP)
        val r = if (at.peeking) 0f else at.radius
        c.drawRoundRect(where, r, r, p)
    }

    private val live = MenuState(
        tone = Tone.LIVE, headline = "R4 · 5769 MHz", detail = "NTSC · 60 fps · 27 dB", radioReady = true, locked = true,
        found = listOf(Found(5769.2, "R4", 31f, "NTSC"), Found(5843.0, "R6", 22f, "NTSC"), Found(5760.4, "F2", 12f, "PAL")),
        stats = "USB 3 · 40 MS/s · processing 46% · 0 dropped · 0 restarts",
    )

    private fun shoot(name: String, tab: Tab, state: MenuState, w: Int, h: Int, setup: Settings.() -> Unit = {},
                      then: (MenuView) -> Unit = {}) {
        val activity = Robolectric.buildActivity(Activity::class.java).setup().get()
        val prefs = activity.getSharedPreferences("t-$name", Context.MODE_PRIVATE)
        prefs.edit().clear().commit()
        val settings = Settings(prefs).apply(setup)
        val quiet = Quiet()
        val menu = MenuView(activity, settings, quiet)
        activity.setContentView(menu)
        menu.render(state)
        menu.showTab(tab, animate = false)
        shadowOf(Looper.getMainLooper()).idle()
        layout(menu, w, h)
        shadowOf(Looper.getMainLooper()).idle()
        layout(menu, w, h)
        then(menu)
        shadowOf(Looper.getMainLooper()).idleFor(java.time.Duration.ofMillis(600))   // let a fade finish
        // the menu on its own, as the window has it (see-through where the preview cleared its hole), then over the
        // black screen and the stand-in, as the compositor lays the window over the video
        val window = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        window.eraseColor(Color.TRANSPARENT)
        menu.draw(Canvas(window))
        val bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        bitmap.eraseColor(Color.BLACK)
        val canvas = Canvas(bitmap)
        standIn(canvas, quiet)
        canvas.drawBitmap(window, 0f, 0f, null)
        File(outDir, "$name.png").outputStream().use { bitmap.compress(Bitmap.CompressFormat.PNG, 100, it) }
    }

    private fun layout(v: View, w: Int, h: Int) {
        v.measure(View.MeasureSpec.makeMeasureSpec(w, View.MeasureSpec.EXACTLY), View.MeasureSpec.makeMeasureSpec(h, View.MeasureSpec.EXACTLY))
        v.layout(0, 0, w, h)
    }

    private val tuned: Settings.() -> Unit = { band = 'R'; channelMhz = 5769 }

    // ---------------------------------------------------------------- phone, 2400 x 1080 at 420 dpi
    @Test @Config(qualifiers = PHONE)
    fun channelLive() = shoot("channel_live", Tab.CHANNEL, live, 2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun channelScanning() = shoot("channel_scanning", Tab.CHANNEL, MenuState(
        tone = Tone.BUSY, headline = "Scanning", detail = "Looking for video on every channel.", radioReady = true,
        scanning = true, scanStep = 7, scanSteps = 29), 2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun channelNoAdapter() = shoot("channel_noadapter", Tab.CHANNEL, MenuState(
        tone = Tone.IDLE, headline = "No adapter", detail = "Plug the Wi-Fi adapter into the phone's USB port."), 2400, 1080)

    @Test @Config(qualifiers = PHONE)
    fun channelError() = shoot("channel_error", Tab.CHANNEL, MenuState(
        tone = Tone.WARN, headline = "Trying again",
        detail = "The radio did not start: bulk transfer timed out while loading firmware", action = "Try now"), 2400, 1080)

    /** Nothing to say along the top: the name of the app there instead. */
    @Test @Config(qualifiers = PHONE)
    fun channelQuiet() = shoot("channel_quiet", Tab.CHANNEL, MenuState(tone = Tone.BUSY, headline = "Working", radioReady = true),
        2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun view() = shoot("view", Tab.VIEW, live, 2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun viewPeek() = shoot("view_peek", Tab.VIEW, live, 2400, 1080, { tuned(); scale = 0.8f }) { it.peekSizeRow(true) }

    /** The one thing on screen when the picture is tapped. */
    @Test @Config(qualifiers = PHONE)
    fun hint() {
        val activity = Robolectric.buildActivity(Activity::class.java).setup().get()
        val hint = MenuHint(activity)
        hint.measure(View.MeasureSpec.UNSPECIFIED, View.MeasureSpec.UNSPECIFIED)
        hint.layout(0, 0, hint.measuredWidth, hint.measuredHeight)
        val bitmap = Bitmap.createBitmap(hint.measuredWidth, hint.measuredHeight, Bitmap.Config.ARGB_8888)
        bitmap.eraseColor(Color.TRANSPARENT)
        hint.draw(Canvas(bitmap))
        File(outDir, "hint.png").outputStream().use { bitmap.compress(Bitmap.CompressFormat.PNG, 100, it) }
    }

    @Test @Config(qualifiers = PHONE)
    fun viewGoggles() = shoot("view_goggles", Tab.VIEW, live, 2400, 1080, {
        tuned(); goggles = true; wide = true; fit = Settings.CROP; scale = 0.85f; eyeSpacing = 0.3f
    })

    @Test @Config(qualifiers = PHONE)
    fun picture() = shoot("picture", Tab.PICTURE, live, 2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun pictureNoComb() = shoot("picture_nocomb", Tab.PICTURE, live, 2400, 1080, { tuned(); lumaComb = false })

    @Test @Config(qualifiers = PHONE)
    fun signal() = shoot("signal", Tab.SIGNAL, live, 2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun signalDiversity() = shoot("signal_diversity", Tab.SIGNAL, live.copy(
        stats = "USB 3 · 40 MS/s · processing 58% · 0 dropped · 0 restarts\nAntenna A 24 dB · B 19 dB · together 26 dB"),
        2400, 1080, { tuned(); diversity = true })

    @Test @Config(qualifiers = PHONE)
    fun signalManualGain() = shoot("signal_gain", Tab.SIGNAL, live, 2400, 1080, { tuned(); gainAuto = false; gainDb = 24f })

    /** A band picture as the radio draws it: the floor at 0 dB with a little noise, a hump for each carrier. */
    private fun spectrumOf(vararg carriers: Carrier, at: Double = 0.0, sweeps: Int = 2, lo: Int = 5635, hi: Int = 5955): Spectrum {
        val rnd = java.util.Random(7)
        val db = FloatArray((hi - lo) * 2 + 1) { i ->
            val f = lo + i * 0.5
            var v = (rnd.nextGaussian() * 0.7).toFloat()
            for (c in carriers) {
                val off = (f - c.mhz) / (c.widthMhz / 2)
                // digital (18 MHz and wider): a flat top with steep sides; FM video: a broad hump with the sync tip's
                // shoulder on one side
                val hump = if (c.widthMhz >= 18) c.snrDb * Math.exp(-Math.pow(Math.abs(off), 8.0) * 1.2)
                else c.snrDb * Math.exp(-off * off * 1.6) + 0.25 * c.snrDb * Math.exp(-(off + 0.7) * (off + 0.7) * 6)
                v = maxOf(v, (hump + rnd.nextGaussian() * 0.6).toFloat())
            }
            v
        }
        return Spectrum(lo.toDouble(), 0.5, db, carriers.toList(), at, 33.0, sweeps)
    }

    private val onAir = arrayOf(Carrier(5658.4, 11.0, 18f), Carrier(5712.0, 9.0, 9f), Carrier(5769.2, 13.0, 31f), Carrier(5880.6, 12.0, 24f))

    private val sweeping = MenuState(tone = Tone.BUSY, headline = "Spectrum", detail = "Picture paused while sweeping.",
        radioReady = true, sweeping = true, spectrum = spectrumOf(*onAir, at = 5805.0))

    @Test @Config(qualifiers = PHONE)
    fun spectrumSweeping() = shoot("spectrum_sweeping", Tab.SPECTRUM, sweeping, 2400, 1080, tuned)

    /** A channel picked on it: the sweep is over, the last one stays, the picked channel is marked. */
    @Test @Config(qualifiers = PHONE)
    fun spectrumPicked() = shoot("spectrum_picked", Tab.SPECTRUM, live.copy(spectrum = spectrumOf(*onAir)), 2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun spectrumFirstPass() = shoot("spectrum_first_pass", Tab.SPECTRUM, sweeping.copy(spectrum = Spectrum(5635.0, 0.5,
        spectrumOf(*onAir).db.mapIndexed { i, v -> if (i < 260) v else Float.NaN }.toFloatArray(), emptyList(), 5770.0, 33.0, 0)),
        2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun spectrumNothing() = shoot("spectrum_nothing", Tab.SPECTRUM, sweeping.copy(spectrum = spectrumOf(at = 5700.0)), 2400, 1080, tuned)

    @Test @Config(qualifiers = PHONE)
    fun spectrumLowBand() = shoot("spectrum_lowband", Tab.SPECTRUM, sweeping.copy(spectrum = spectrumOf(
        Carrier(5473.3, 12.0, 22f), at = 5500.0, lo = 5340, hi = 5640)), 2400, 1080, { band = 'L'; channelMhz = 5436 })

    /** A field with everything flying: analog on R1, DJI V1/V2 on CH3 and CH8, HDZero on F4 (and R7), O4 on its CH1. */
    private val mixed = arrayOf(Carrier(5658.4, 11.0, 18f), Carrier(5735.1, 19.0, 26f), Carrier(5768.6, 19.0, 22f),
        Carrier(5800.3, 17.0, 29f), Carrier(5839.2, 19.0, 15f), Carrier(5880.4, 17.0, 24f))

    @Test @Config(qualifiers = PHONE)
    fun spectrumDji() = shoot("spectrum_dji", Tab.SPECTRUM, sweeping.copy(spectrum = spectrumOf(*mixed, at = 5700.0)),
        2400, 1080, { tuned(); spectrumSystem = ChannelSystem.DJI })

    /** DJI V1/V2 pilots, one of them at 50 Mbps (40 MHz wide, its CH2 on the 25 Mbps CH4's frequency). */
    @Test @Config(qualifiers = PHONE)
    fun spectrumDji50() = shoot("spectrum_dji_50", Tab.SPECTRUM, sweeping.copy(spectrum = spectrumOf(
        Carrier(5658.4, 11.0, 18f), Carrier(5735.1, 19.0, 26f), Carrier(5770.2, 39.0, 21f), Carrier(5878.3, 19.5, 24f), at = 5850.0)),
        2400, 1080, { tuned(); spectrumSystem = ChannelSystem.DJI })

    @Test @Config(qualifiers = PHONE)
    fun spectrumDjiO4() = shoot("spectrum_dji_o4", Tab.SPECTRUM, sweeping.copy(spectrum = spectrumOf(*mixed, at = 5700.0)),
        2400, 1080, { tuned(); spectrumSystem = ChannelSystem.DJI; djiModel = 2 })

    @Test @Config(qualifiers = PHONE)
    fun spectrumHdzero() = shoot("spectrum_hdzero", Tab.SPECTRUM, sweeping.copy(spectrum = spectrumOf(*mixed, at = 5700.0)),
        2400, 1080, { tuned(); spectrumSystem = ChannelSystem.HDZERO })

    @Test @Config(qualifiers = SMALL)
    fun smallSpectrumHdzero() = shoot("small_spectrum_hdzero", Tab.SPECTRUM, sweeping.copy(spectrum = spectrumOf(*mixed)),
        1600, 720, { tuned(); spectrumSystem = ChannelSystem.HDZERO })

    @Test @Config(qualifiers = PHONE)
    fun spectrumNoAdapter() = shoot("spectrum_noadapter", Tab.SPECTRUM, MenuState(
        tone = Tone.IDLE, headline = "No adapter", detail = "Plug the Wi-Fi adapter into the phone's USB port."), 2400, 1080)

    // ---------------------------------------------------------------- small phone, 1600 x 720 at 320 dpi
    @Test @Config(qualifiers = SMALL)
    fun smallSpectrum() = shoot("small_spectrum", Tab.SPECTRUM, sweeping, 1600, 720, { band = 'B'; channelMhz = 5771 })

    @Test @Config(qualifiers = SMALL)
    fun smallChannel() = shoot("small_channel", Tab.CHANNEL, live, 1600, 720, tuned)

    @Test @Config(qualifiers = SMALL)
    fun smallPicture() = shoot("small_picture", Tab.PICTURE, live, 1600, 720, tuned)

    /** A short screen and a long status: the line of the status along the top, the tabs and the preview all fit. */
    @Test @Config(qualifiers = SMALL)
    fun smallNoAdapter() = shoot("small_noadapter", Tab.CHANNEL, MenuState(
        tone = Tone.IDLE, headline = "No adapter", detail = "Plug the Wi-Fi adapter into the phone's USB port."), 1600, 720)

    @Test @Config(qualifiers = SMALL)
    fun smallError() = shoot("small_error", Tab.VIEW, MenuState(
        tone = Tone.WARN, headline = "Trying again",
        detail = "The radio stopped: no samples for 4 seconds. If this goes on, unplug the adapter and plug it in again.",
        action = "Try now"), 1600, 720)

    /** Narrower still: the same screen with a camera cutout taking 68 dp of its width. */
    @Test @Config(qualifiers = "w732dp-h360dp-land-xhdpi")
    fun narrowView() = shoot("narrow_view", Tab.VIEW, MenuState(
        tone = Tone.IDLE, headline = "No adapter", detail = "Plug the Wi-Fi adapter into the phone's USB port."), 1464, 720,
        { goggles = true })

    // ---------------------------------------------------------------- tablet, 2560 x 1600 at 320 dpi
    @Test @Config(qualifiers = TABLET)
    fun tabletChannel() = shoot("tablet_channel", Tab.CHANNEL, live, 2560, 1600, tuned)

    companion object {
        const val PHONE = "w914dp-h411dp-land-420dpi"
        const val SMALL = "w800dp-h360dp-land-xhdpi"
        const val TABLET = "w1280dp-h800dp-land-xhdpi"
    }
}
