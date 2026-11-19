package com.kepyur.app

import android.annotation.SuppressLint
import android.app.Activity
import android.content.Intent
import android.graphics.Color
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.Gravity
import android.view.MotionEvent
import android.view.Surface
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.view.WindowManager
import android.view.animation.DecelerateInterpolator
import android.widget.FrameLayout
import org.json.JSONObject
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * kepyur: one screen. The picture fills it; the menu ([MenuView]) lies over the picture, blurred behind it,
 * and is open when the app starts. With the menu closed there is nothing on screen but the picture: a tap
 * brings up a MENU button for three seconds.
 *
 * USB stream test (for the first hardware check):  adb shell am start -n com.kepyur.app/.MainActivity --es mode usbtest
 */
class MainActivity : Activity(), MenuView.Listener, UsbLink.Events {
    private val ui = Handler(Looper.getMainLooper())
    private val engine = Engine.get()
    private lateinit var settings: Settings
    private lateinit var link: UsbLink

    private lateinit var root: FrameLayout
    private lateinit var video: SurfaceView
    private lateinit var menu: MenuView
    private lateinit var hint: MenuHint
    private var menuOpen = true
    private var foreground = false

    // what the radio last said
    private var mode = ""
    private var found: List<Found> = emptyList()
    private var noVideoFound = false       // the last scan came back empty
    private var lastDropped = 0L
    private var overloadedUntil = 0L
    private var frameRateFor = ""
    private var appliedRate = 0
    private var scanOnReturn = false       // a scan is owed: the first one, or one the background cut short
    private var statusBroken = false

    // USB stream test
    private val testPlan = ArrayDeque<Pair<Int, Double>>()
    private var testRunning = false
    private val testResults = mutableListOf<String>()
    private var testRequested = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        settings = Settings(this)
        engine.init(getExternalFilesDir(null)!!.absolutePath + "/kepyur.log")
        engine.log("start, ${Build.MANUFACTURER} ${Build.MODEL} Android ${Build.VERSION.RELEASE}")
        applyPicture()
        applySignal()
        applyView()
        engine.setBackdrop(true, true)
        buildUi()
        link = UsbLink(this, engine, this)
        handleIntent(intent)
        link.start()
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        handleIntent(intent)
        if (intent.action == UsbManager.ACTION_USB_DEVICE_ATTACHED) link.onAttachedIntent()
    }

    /** adb shell am start -n com.kepyur.app/.MainActivity [--es mode usbtest] [--ef gain 30] [--ef record 3] [--ez testcard true] */
    private fun handleIntent(i: Intent?) {
        i ?: return
        if (i.hasExtra("testcard")) engine.testCard(i.getBooleanExtra("testcard", false))
        if (i.getStringExtra("mode") == "usbtest") {
            testRequested = true
            if (::link.isInitialized && link.state == UsbLink.State.RUNNING) startTest()
        }
        if (i.hasExtra("gain")) {   // the same as the menu's Gain: below 0 automatic, else that many dB
            val g = i.getFloatExtra("gain", -1f)
            settings.gainAuto = g < 0
            if (g >= 0) settings.gainDb = g.coerceAtMost(Settings.GAIN_MAX)
            settings.save()
            engine.log("debug: gain ${if (g < 0) "automatic" else "$g dB"}")
            applySignal()
            retune()
        }
        if (i.hasExtra("record")) {
            val secs = i.getFloatExtra("record", 3f).toDouble()
            val name = "rec-" + java.text.SimpleDateFormat("HHmmss", java.util.Locale.US).format(java.util.Date())
            engine.record(secs, getExternalFilesDir(null)!!.absolutePath + "/" + name)
        }
    }

    override fun onStart() {
        super.onStart()
        foreground = true
        ui.removeCallbacks(closeInBackground)
        ui.removeCallbacks(poll)
        ui.post(poll)
        link.start()   // again, if the adapter was let go in the background (onRadioUp then tunes)
        // back from the background, where decoding was stopped
        if (link.state == UsbLink.State.RUNNING && !testRunning) {
            if (scanOnReturn) onScan() else if (mode == "idle") tune(fresh = true)
        }
    }

    override fun onResume() {
        super.onResume()
        hideSystemBars()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) hideSystemBars()
    }

    override fun onStop() {
        foreground = false
        ui.removeCallbacks(poll)
        // nobody is watching: no reason to keep two cores decoding
        if (link.state == UsbLink.State.RUNNING && !testRunning) {
            if (mode == "scan") scanOnReturn = true   // cut short: start it over when the app is back
            engine.idle()
            mode = "idle"
        }
        // ... nor, after a while, to keep the adapter streaming and warm: it is opened again on return
        if (!testRunning) ui.postDelayed(closeInBackground, BACKGROUND_CLOSE_MS)
        super.onStop()
    }

    private val closeInBackground = Runnable {
        engine.log("in the background for ${BACKGROUND_CLOSE_MS / 1000} s: closing the adapter")
        link.stop()
        mode = ""
    }

    override fun onDestroy() {
        ui.removeCallbacksAndMessages(null)
        link.stop()
        super.onDestroy()
    }

    /** Back goes up: from the picture to the menu, from the menu out of the app. */
    @Deprecated("Deprecated in Java")
    override fun onBackPressed() {
        if (menuOpen) {
            @Suppress("DEPRECATION") super.onBackPressed()
        } else openMenu()
    }

    // ---------------------------------------------------------------------------------------- settings -> native
    private fun applyPicture() {
        engine.setPicture(settings.lumaComb, settings.lumaComb && settings.cleanOsd, settings.smoothLines, settings.sparkleRepair)
        engine.setComb(settings.colourComb)
    }

    /** True when something changed that only a new tune applies. */
    private fun applySignal(): Boolean {
        // only when it changed: setting the rate also forgets that 40 had to fall back to 20
        val rate = if (settings.rate40) 1 else 2
        if (rate != appliedRate) engine.setWatchRate(rate)
        appliedRate = rate
        engine.setGain(if (settings.gainAuto) -1.0 else settings.gainDb.toDouble())
        engine.setDiversity(settings.diversity)
        return engine.setOffsetTuning(settings.offsetTune)
    }

    private fun applyView() =
        engine.setView(settings.aspect, settings.fit, settings.scale, settings.goggles, settings.eyeShift)

    /** The frequency for the chosen channel: where a scan measured its transmitter, if it found one, else the nominal one. */
    private fun channelHz(): Double {
        val mhz = settings.channelMhz
        if (mhz <= 0) return 0.0
        val seen = found.filter { abs(it.mhz - mhz) < 4 }.maxByOrNull { it.snrDb }
        return (seen?.mhz ?: mhz.toDouble()) * 1e6
    }

    private fun tune(fresh: Boolean) {
        if (link.state != UsbLink.State.RUNNING || testRunning) return
        val hz = channelHz()
        if (hz <= 0) return
        noVideoFound = false
        engine.watch(hz, fresh)
        mode = "watch"
    }

    /** The same channel again, for a setting that only takes effect on a tune. */
    private fun retune() {
        if (mode == "watch") tune(fresh = false)
    }

    // ---------------------------------------------------------------------------------------- MenuView.Listener
    override fun onTune() = tune(fresh = true)

    override fun onScan() {
        if (link.state != UsbLink.State.RUNNING || testRunning) return
        scanOnReturn = false
        noVideoFound = false
        engine.scan(channelHz())
        mode = "scan"
        refresh()
    }

    override fun onView() = applyView()

    override fun onPicture(rebuild: Boolean) {
        applyPicture()
        if (rebuild) retune()
    }

    override fun onSignal(retune: Boolean) {
        applySignal()
        if (retune) retune()
    }

    /** A view slider is being dragged: the picture sharp under it, so the change can be judged. */
    override fun onPeek(on: Boolean) = engine.setBackdrop(!on, false)

    override fun onStatusAction() = link.retryNow()

    override fun onClose() = closeMenu()

    // ---------------------------------------------------------------------------------------- UsbLink.Events
    override fun onLinkChanged() {
        if (link.state != UsbLink.State.RUNNING) {
            mode = ""
            testRunning = false
        }
        refresh()
    }

    override fun onRadioUp() {
        applyPicture()
        applySignal()
        lastDropped = 0
        when {
            testRequested -> startTest()
            !foreground -> {   // it came up with nobody watching: wait, and on return tune, or scan if this is the first run
                engine.idle()
                mode = "idle"
                scanOnReturn = settings.channelMhz <= 0
            }
            settings.channelMhz > 0 -> tune(fresh = true)
            else -> onScan()   // first run: find something to show
        }
        refresh()
    }

    // ---------------------------------------------------------------------------------------- USB stream test
    private fun startTest() {
        testRequested = false
        testPlan.clear(); testResults.clear()
        testPlan.add(1 to 60.0)   // 40 MS/s for 60 s
        testPlan.add(2 to 60.0)   // 20 MS/s for 60 s
        nextTest()
    }

    private fun nextTest() {
        val t = testPlan.removeFirstOrNull()
        if (t == null) {
            testRunning = false
            engine.log("usbtest: all done: ${testResults.joinToString(" | ")}")
            engine.log("usbtest: now decoding for the rest of the session")
            applySignal()
            if (settings.channelMhz > 0) tune(fresh = true) else onScan()
            return
        }
        testRunning = true
        engine.log("usbtest: rate_sel ${t.first} for ${t.second} s")
        engine.streamTest(t.first, t.second)
        mode = "test"
    }

    // ---------------------------------------------------------------------------------------- status
    private val poll = object : Runnable {
        override fun run() {
            refresh()
            ui.postDelayed(this, 250)
        }
    }

    private fun refresh() {
        if (!::menu.isInitialized) return
        val st = if (link.state == UsbLink.State.RUNNING) radioState() else linkState()
        if (menuOpen) menu.render(st)
    }

    /**
     * A number from the status, or [or] when it is missing or not a finite number. (The native side prints
     * doubles as they come, and Android's JSON reader takes a bare nan or inf for a string without complaint.)
     */
    private fun JSONObject.num(key: String, or: Double = 0.0): Double = optDouble(key, or).let { if (it.isFinite()) it else or }

    private fun linkState(): MenuState = when (link.state) {
        UsbLink.State.NO_ADAPTER -> MenuState(Tone.IDLE, "No adapter", "Plug the Wi-Fi adapter into the phone's USB port.")
        UsbLink.State.PERMISSION -> MenuState(Tone.BUSY, "Waiting for permission", "Allow Kepyur to use the adapter.")
        UsbLink.State.DENIED -> MenuState(Tone.WARN, "Permission needed", "Kepyur cannot use the adapter without it.", "Ask again")
        UsbLink.State.SWITCHING -> MenuState(Tone.BUSY, "Switching to USB 3", "The adapter restarts itself. This takes a few seconds.")
        UsbLink.State.OPENING -> MenuState(Tone.BUSY, "Starting the radio", link.message.ifEmpty { "About five seconds." })
        UsbLink.State.WAITING -> MenuState(Tone.WARN, "Trying again", link.message, "Try now")
        UsbLink.State.FAILED -> MenuState(Tone.ERROR, "Adapter problem", link.message, "Try again")
        UsbLink.State.USB2 -> MenuState(Tone.WARN, "USB 2 link",
            "Video needs USB 3. Plug the adapter straight into the phone with a USB 3 (OTG) adapter or cable.", "Try again")
        UsbLink.State.RUNNING -> MenuState()
    }

    private fun radioState(): MenuState {
        // a number the native side could not print as JSON (nan, inf) must not take the app down with it
        val s = try {
            JSONObject(engine.status())
        } catch (e: org.json.JSONException) {
            if (!statusBroken) engine.log("status: unreadable ($e)")
            statusBroken = true
            return MenuState(Tone.BUSY, "Working", radioReady = !testRunning, found = found)
        }
        statusBroken = false
        link.check(s)
        if (link.state != UsbLink.State.RUNNING) return linkState()
        val was = mode
        mode = s.optString("mode")
        val v = s.optJSONObject("video")
        val locked = v?.optBoolean("locked") ?: false
        val watchHz = s.num("watch_hz")

        found = buildList {
            val arr = s.optJSONArray("pilots") ?: return@buildList
            for (i in 0 until arr.length()) {
                val p = arr.optJSONObject(i) ?: continue
                val mhz = p.num("mhz")
                if (mhz > 0) add(Found(mhz, p.optString("name"), p.num("snr").toFloat(), p.optString("std")))
            }
        }
        // a scan has just ended: it left the radio on the channel it judged best, and the picker follows
        if (was == "scan" && mode != "scan") {
            noVideoFound = found.isEmpty()
            if (mode == "watch") adoptChannel(watchHz / 1e6)
        }
        if (testRunning && mode == "idle") {
            s.optJSONObject("test")?.let { if (it.optBoolean("done")) { testResults.add(it.toString()); nextTest() } }
        }
        if (mode == "watch" && locked) matchDisplayTo(v?.optString("std") ?: "")

        // is the phone keeping up? (dropped blocks, or a thread with no time to spare)
        val now = android.os.SystemClock.elapsedRealtime()
        val dropped = s.optLong("dropped") + s.optLong("decode_dropped")
        val load = maxOf(s.num("dsp_load"), s.num("decode_load"))
        if (mode == "watch" && (dropped > lastDropped || load > 0.95)) overloadedUntil = now + 5000
        lastDropped = dropped
        val overloaded = mode == "watch" && now < overloadedUntil

        val name = Channels.nearest(watchHz / 1e6, settings.band, 4.0)?.name
        val where = (if (name != null) "$name · " else "") + "${(watchHz / 1e6).roundToInt()} MHz"   // finite: num()
        val stats = buildString {
            append(if (s.optBoolean("usb3")) "USB 3" else "USB 2")
            val msps = s.num("watch_msps")
            if (msps > 0) append(" · %.0f MS/s".format(msps))
            if (s.optBoolean("fallback")) append(" (40 kept dropping out)")
            append(" · processing %.0f%%".format(load * 100))
            append(" · $dropped dropped · ${s.optLong("restarts")} restarts")
            s.optJSONObject("antennas")?.let { a ->
                if (a.optBoolean("combine")) append("\nAntenna A %.0f dB · B %.0f dB · together %.0f dB".format(
                    a.num("snr_a"), a.num("snr_b"), a.num("snr_c")))
            }
        }
        val base = MenuState(radioReady = !testRunning, scanning = mode == "scan", scanStep = s.optInt("scan_step"),
            scanSteps = s.optInt("scan_steps"), found = found, stats = stats)
        return when (mode) {
            "scan" -> base.copy(tone = Tone.BUSY, headline = "Scanning", detail = "Looking for video on every channel.")
            "test" -> {
                val t = s.optJSONObject("test")
                base.copy(tone = Tone.BUSY, headline = "USB test", detail = t?.let {
                    "%.0f MS/s · %.0f s\n%d of %d blocks · %d gaps · %d restarts".format(it.num("rate_msps"), it.num("seconds"),
                        it.optLong("banks"), it.optLong("expected"), it.optLong("seq_gaps"), it.optLong("restarts"))
                } ?: "Starting…")
            }
            "watch" -> when {
                overloaded -> base.copy(tone = Tone.WARN, headline = where,
                    detail = "The phone is not keeping up. Turn off Clean OSD edges, Colour comb or Diversity, or use sample rate 20.")
                locked -> base.copy(tone = Tone.LIVE, headline = where, detail = buildString {
                    append(v?.optString("std") ?: "")
                    if (v?.optBoolean("colour") == false) append(" · black and white")
                    append(" · %.0f fps".format(v?.num("fps") ?: 0.0))
                    if (s.has("snr_db")) append(" · %.0f dB".format(s.num("snr_db")))
                })
                else -> base.copy(tone = Tone.BUSY, headline = where, detail = "Looking for a picture…")
            }
            else -> base.copy(tone = Tone.IDLE, headline = "Ready",
                detail = if (noVideoFound) "The scan found no video. Pick a channel, or scan again." else "Pick a channel, or scan.")
        }
    }

    /** The radio is on [mhz] by its own choice (after a scan): the picker and the saved channel follow. */
    private fun adoptChannel(mhz: Double) {
        if (!mhz.isFinite() || mhz <= 0) return
        val ch = Channels.nearest(mhz, settings.band)
        val to = ch?.mhz ?: mhz.roundToInt()
        if (to == settings.channelMhz && (ch == null || ch.band == settings.band)) return
        settings.channelMhz = to
        if (ch != null) settings.band = ch.band
        settings.save()
        menu.syncChannel()
    }

    /** Ask for a display mode that fits the camera's field rate, so frames do not beat against the refresh. */
    private fun matchDisplayTo(std: String) {
        if (Build.VERSION.SDK_INT < 30 || std == frameRateFor) return
        val rate = when (std) { "PAL" -> 50f; "NTSC" -> 59.94f; else -> return }
        val surface = video.holder.surface ?: return
        if (!surface.isValid) return
        frameRateFor = std
        try {
            surface.setFrameRate(rate, Surface.FRAME_RATE_COMPATIBILITY_FIXED_SOURCE)
        } catch (e: Exception) {
            engine.log("setFrameRate: $e")
        }
    }

    // ---------------------------------------------------------------------------------------- UI
    private fun dp(v: Float) = v * resources.displayMetrics.density

    @SuppressLint("ClickableViewAccessibility")
    private fun buildUi() {
        root = FrameLayout(this).apply { setBackgroundColor(Color.BLACK) }
        video = SurfaceView(this)
        video.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(h: SurfaceHolder) {
                frameRateFor = ""
                engine.setSurface(h.surface)
            }
            override fun surfaceChanged(h: SurfaceHolder, f: Int, w: Int, ht: Int) = engine.redraw()
            override fun surfaceDestroyed(h: SurfaceHolder) = engine.setSurface(null)
        })
        root.addView(video, FrameLayout.LayoutParams(-1, -1))

        hint = MenuHint(this).apply {
            alpha = 0f
            visibility = View.GONE
            setOnClickListener { openMenu() }
        }
        root.addView(hint, FrameLayout.LayoutParams(-2, -2, Gravity.BOTTOM or Gravity.CENTER_HORIZONTAL)
            .apply { bottomMargin = dp(30f).roundToInt() })

        menu = MenuView(this, settings, this)
        root.addView(menu, FrameLayout.LayoutParams(-1, -1))

        // a tap on the picture: the MENU button, for three seconds
        var downX = 0f
        var downY = 0f
        root.setOnTouchListener { _, e ->
            when (e.actionMasked) {
                MotionEvent.ACTION_DOWN -> { downX = e.x; downY = e.y }
                MotionEvent.ACTION_UP -> if (!menuOpen && abs(e.x - downX) < dp(16f) && abs(e.y - downY) < dp(16f)) showHint()
            }
            true
        }
        setContentView(root)
    }

    private val hideHint = Runnable {
        hint.animate().alpha(0f).setDuration(450).withEndAction { hint.visibility = View.GONE }.start()
    }

    private fun showHint() {
        ui.removeCallbacks(hideHint)
        hint.animate().cancel()
        hint.visibility = View.VISIBLE
        hint.animate().alpha(1f).setDuration(140).start()
        ui.postDelayed(hideHint, HINT_MS)
    }

    private fun openMenu() {
        if (menuOpen) return
        menuOpen = true
        ui.removeCallbacks(hideHint)
        hint.animate().cancel()
        hint.alpha = 0f
        hint.visibility = View.GONE
        engine.setBackdrop(true, false)
        refresh()
        menu.animate().cancel()
        menu.visibility = View.VISIBLE
        menu.alpha = 0f
        menu.translationY = dp(10f)
        menu.animate().alpha(1f).translationY(0f).setDuration(240).setInterpolator(DecelerateInterpolator(1.5f)).withEndAction(null).start()
    }

    private fun closeMenu() {
        if (!menuOpen) return
        menuOpen = false
        engine.setBackdrop(false, false)
        menu.animate().cancel()
        menu.animate().alpha(0f).translationY(dp(10f)).setDuration(200).setInterpolator(DecelerateInterpolator(1.5f))
            .withEndAction { menu.visibility = View.GONE }.start()
    }

    private fun hideSystemBars() {
        window.insetsController?.let {
            it.hide(WindowInsets.Type.systemBars())
            it.systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
    }

    companion object {
        const val HINT_MS = 3000L
        const val BACKGROUND_CLOSE_MS = 30000L
    }
}
