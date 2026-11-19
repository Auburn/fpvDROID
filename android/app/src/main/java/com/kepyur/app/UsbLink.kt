package com.kepyur.app

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import org.json.JSONObject
import java.util.concurrent.Executors

/**
 * The adapter's whole life: found, permitted, switched to USB 3, opened, watched over while it runs, and
 * brought back when anything goes wrong. Nothing here waits for the user: a failed start is tried again by
 * itself, a radio that stops delivering is closed and reopened, and every wait has an end.
 *
 * All of it runs on the main thread except the native calls that block (open, close, the USB 3 switch), which
 * go through one worker thread, in order.
 */
class UsbLink(private val ctx: Context, private val engine: Engine, private val events: Events) {
    interface Events {
        /** [state] or [message] changed. */
        fun onLinkChanged()
        /** The radio has just opened: apply the settings and tune. Also after every automatic reopen. */
        fun onRadioUp()
    }

    enum class State {
        NO_ADAPTER,   // nothing plugged in
        PERMISSION,   // Android is asking the user
        DENIED,       // the user said no
        SWITCHING,    // the adapter is re-enumerating as a USB 3 device
        OPENING,      // chip bring-up (about five seconds)
        RUNNING,
        USB2,         // open, but on a link too slow for video
        WAITING,      // something failed; trying again shortly ([message] says what)
        FAILED,       // something failed that trying again by itself will not fix ([message])
    }

    var state = State.NO_ADAPTER
        private set
    var message = ""
        private set

    private val ui = Handler(Looper.getMainLooper())
    private val worker = Executors.newSingleThreadExecutor()
    private val usb = ctx.getSystemService(Context.USB_SERVICE) as UsbManager
    private val permAction = "com.kepyur.app.USB_PERMISSION"
    private var conn: UsbDeviceConnection? = null
    private var started = false

    /** Bumped whenever the adapter in hand is given up: what an open in flight finds when it lands tells it to stand down. */
    private var generation = 0
    private var failures = 0
    private var switchRequests = 0
    private var switchDeadline = 0L
    // the bank counter of the last status, and when it last moved (a stream that has silently stopped)
    private var lastBanks = -1L
    private var banksMovedAt = 0L
    private var checkedAt = 0L

    fun start() {
        if (started) return
        started = true
        val f = IntentFilter().apply {
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
        }
        if (Build.VERSION.SDK_INT >= 33) {
            ctx.registerReceiver(usbReceiver, f, Context.RECEIVER_EXPORTED)
            ctx.registerReceiver(permReceiver, IntentFilter(permAction), Context.RECEIVER_NOT_EXPORTED)
        } else {
            ctx.registerReceiver(usbReceiver, f)
            ctx.registerReceiver(permReceiver, IntentFilter(permAction))
        }
        connect()
    }

    fun stop() {
        if (!started) return
        started = false
        ui.removeCallbacksAndMessages(null)
        ctx.unregisterReceiver(usbReceiver)
        ctx.unregisterReceiver(permReceiver)
        giveUpAdapter()
    }

    /** The user asked for another go (a tap on the status). */
    fun retryNow() {
        when (state) {
            State.DENIED, State.WAITING, State.FAILED, State.USB2, State.NO_ADAPTER -> {
                ui.removeCallbacks(retry)
                if (state == State.USB2) giveUpAdapter()
                switchRequests = 0
                connect(force = true)
            }
            else -> {}
        }
    }

    /** Called from the app's own launch intent when Android started it for an attached adapter. */
    fun onAttachedIntent() {
        if (state != State.RUNNING && state != State.OPENING) connect(force = true)
    }

    // ---------------------------------------------------------------------------------------- connecting
    private fun ours(d: UsbDevice) = // T4U v3, T3U, T3U Plus; Realtek's own 8812BU id
        (d.vendorId == 0x2357 && d.productId in setOf(0x0115, 0x012d, 0x0138)) ||
            (d.vendorId == 0x0bda && d.productId == 0xb812)

    private fun set(s: State, msg: String = "") {
        if (s == state && msg == message) return
        state = s
        message = msg
        events.onLinkChanged()
    }

    private fun connect(force: Boolean = false) {
        if (!started) return
        if (state == State.OPENING || state == State.RUNNING || state == State.USB2) return
        if (state == State.DENIED && !force) return
        ui.removeCallbacks(retry)
        val d = usb.deviceList.values.firstOrNull { ours(it) }
        if (d == null) {
            // gone because it is coming back as a USB 3 device? then keep saying so until the deadline
            if (SystemClock.elapsedRealtime() < switchDeadline) set(State.SWITCHING) else set(State.NO_ADAPTER)
            return
        }
        if (!usb.hasPermission(d)) {
            set(State.PERMISSION)
            val pi = PendingIntent.getBroadcast(
                ctx, 0, Intent(permAction).setPackage(ctx.packageName),
                PendingIntent.FLAG_MUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
            )
            usb.requestPermission(d, pi)
            return
        }
        val c = usb.openDevice(d)
        if (c == null) {
            failLater("Android would not open the adapter. Another app may be using it.")
            return
        }
        conn = c
        val fd = c.fileDescriptor
        val gen = ++generation
        engine.log("adapter ${d.deviceName} ${"%04x:%04x".format(d.vendorId, d.productId)} fd=$fd")
        set(State.OPENING)
        // bring-up cannot be interrupted from here, but pulling the plug ends it: say so if it drags on
        ui.postDelayed({
            if (gen == generation && state == State.OPENING)
                set(State.OPENING, "This is taking longer than it should. Unplug the adapter and plug it in again.")
        }, SLOW_OPEN_MS)
        val trySwitch = switchRequests < MAX_SWITCH_REQUESTS
        worker.execute {
            val sw = if (trySwitch) engine.usb3Switch(fd) else "skipped"
            if (sw.startsWith("requested")) {
                // the adapter drops off the bus and comes back as SuperSpeed: a new device, a new permission
                ui.post {
                    if (gen != generation) return@post
                    switchRequests++
                    generation++
                    c.close()
                    conn = null
                    switchDeadline = SystemClock.elapsedRealtime() + SWITCH_WAIT_MS
                    set(State.SWITCHING)
                    ui.removeCallbacks(switchWatch)
                    ui.postDelayed(switchWatch, 1000)
                }
                return@execute
            }
            val err = engine.open(fd)
            val fast = err.isEmpty() && engine.superspeed()
            ui.post {
                if (gen != generation) return@post   // unplugged meanwhile: the close queued behind us tidies up
                switchDeadline = 0
                when {
                    err.isNotEmpty() -> failLater("The radio did not start: $err")
                    !fast -> set(State.USB2)
                    else -> {
                        failures = 0
                        lastBanks = -1
                        banksMovedAt = SystemClock.elapsedRealtime()
                        set(State.RUNNING)
                        events.onRadioUp()
                    }
                }
            }
        }
    }

    /** While the adapter re-enumerates: open it as soon as it is back; say so if it never comes. */
    private val switchWatch = object : Runnable {
        override fun run() {
            if (state != State.SWITCHING) return
            if (usb.deviceList.values.any { ours(it) }) {
                connect()
                if (state != State.SWITCHING) return
            }
            if (SystemClock.elapsedRealtime() >= switchDeadline) {
                switchDeadline = 0
                set(State.FAILED, "The adapter did not come back after switching to USB 3. Unplug it and plug it in again.")
                return
            }
            ui.postDelayed(this, 1000)
        }
    }

    private val retry = Runnable { connect(force = true) }

    /** Close what there is, say what went wrong, and try again shortly. */
    private fun failLater(msg: String) {
        engine.log("link: $msg (failure ${failures + 1})")
        giveUpAdapter()
        failures++
        set(State.WAITING, msg)
        ui.removeCallbacks(retry)
        ui.postDelayed(retry, if (failures <= 1) FIRST_RETRY_MS else RETRY_MS)
    }

    /** Stop using the adapter in hand. The connection closes after the native side has let go of its fd. */
    private fun giveUpAdapter() {
        generation++
        val c = conn
        conn = null
        worker.execute {
            engine.close()
            c?.close()
        }
    }

    // ---------------------------------------------------------------------------------------- watching over it
    /**
     * Once per status poll while [State.RUNNING]: is the radio still alive? A receiver that has given up
     * ("lost", "error", or its thread gone), or one that still says it is streaming while no samples have
     * arrived for [STALL_MS], is closed and reopened; [Events.onRadioUp] then tunes the channel again.
     */
    fun check(status: JSONObject) {
        if (state != State.RUNNING) return
        val now = SystemClock.elapsedRealtime()
        val banks = status.optLong("banks", -1)
        // polling paused for a while (the app was in the background): the clock starts again
        if (now - checkedAt > 1500) banksMovedAt = now
        checkedAt = now
        if (banks != lastBanks) {
            lastBanks = banks
            banksMovedAt = now
        }
        val rx = status.optString("rx_state")
        val why = when {
            status.optBoolean("dead") || rx == "lost" || rx == "error" ->
                status.optString("rx_error").ifEmpty { status.optString("last_fault") }.ifEmpty { "the adapter stopped answering" }
            now - banksMovedAt > STALL_MS -> "no samples for ${STALL_MS / 1000} seconds"
            else -> return
        }
        failLater("The radio stopped: $why")
    }

    // ---------------------------------------------------------------------------------------- broadcasts
    private val usbReceiver = object : BroadcastReceiver() {
        override fun onReceive(c: Context, i: Intent) {
            val d: UsbDevice? = if (Build.VERSION.SDK_INT >= 33)
                i.getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
            else @Suppress("DEPRECATION") i.getParcelableExtra(UsbManager.EXTRA_DEVICE)
            if (d == null || !ours(d)) return
            when (i.action) {
                UsbManager.ACTION_USB_DEVICE_DETACHED -> {
                    val switching = SystemClock.elapsedRealtime() < switchDeadline
                    engine.log("adapter detached${if (switching) " (USB 3 switch)" else ""}")
                    ui.removeCallbacks(retry)
                    giveUpAdapter()
                    failures = 0
                    if (switching) set(State.SWITCHING) else { switchRequests = 0; set(State.NO_ADAPTER) }
                }
                UsbManager.ACTION_USB_DEVICE_ATTACHED -> {
                    engine.log("adapter attached")
                    if (state != State.RUNNING && state != State.OPENING) connect(force = true)
                }
            }
        }
    }

    private val permReceiver = object : BroadcastReceiver() {
        override fun onReceive(c: Context, i: Intent) {
            val ok = i.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
            engine.log("usb permission ${if (ok) "granted" else "denied"}")
            if (state != State.PERMISSION) return
            if (ok) connect(force = true) else set(State.DENIED)
        }
    }

    companion object {
        const val FIRST_RETRY_MS = 2500L
        const val RETRY_MS = 5000L
        const val SWITCH_WAIT_MS = 14000L   // re-enumeration takes "well over 4 s" on some controllers
        const val STALL_MS = 4000L
        const val SLOW_OPEN_MS = 25000L
        const val MAX_SWITCH_REQUESTS = 2   // a link without USB 3 lines would otherwise bounce for ever
    }
}
