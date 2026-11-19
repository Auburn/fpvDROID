package com.kepyur.app

import android.os.SystemClock
import com.kepyur.app.UsbLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import java.time.Duration

/**
 * The adapter's life (UsbLink) inside the running app: the activity polls the fake's status every 250 ms and
 * UsbLink acts on it. Time is the main looper's clock; the worker thread is waited for by [settle].
 */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35], qualifiers = AppHarness.PHONE, shadows = [RecordingUsbManager::class])
class UsbLinkScenariosTest : AppHarness() {
    private val f2 = 5760e6

    /** F2 saved, adapter plugged in with permission, radio up and watching. */
    private fun running(): android.hardware.usb.UsbDevice {
        saveChannel(5760, 'F')
        val d = adapter()
        plug(d, broadcast = false)
        launch()
        assertEquals(State.RUNNING, link.state)
        assertEquals(FakeEngine.Call("watch", listOf(f2, true)), fake.last("watch"))
        return d
    }

    // ------------------------------------------------------------------------------------------ 4. open fails
    @Test
    fun openFailureWaitsRetriesAfter2_5Then5SecondsAndTunesOnceItWorks() {
        saveChannel(5760, 'F')
        fake.openResults += listOf("boom", "boom")
        plug(adapter(), broadcast = false)
        launch()
        val failedAt = SystemClock.elapsedRealtime()

        assertEquals(State.WAITING, link.state)
        assertEquals("The radio did not start: boom", link.message)
        advance(300)
        assertEquals("Trying again", headline)
        assertEquals("The radio did not start: boom", detail)
        assertEquals("Try now", action)
        assertEquals(1, fake.count("open"))
        assertEquals(1, fake.count("close"))
        assertEquals(listOf("open", "close"), fake.radioCalls().map { it.name }.filter { it == "open" || it == "close" })
        assertEquals(0, fake.count("watch"))

        advanceUntil("the first retry", 6000) { fake.count("open") == 2 }
        val first = SystemClock.elapsedRealtime() - failedAt
        val secondFailedAt = SystemClock.elapsedRealtime()
        assertTrue("first retry after $first ms, expected about 2500", first in 2450..2600)
        assertEquals(State.WAITING, link.state)
        assertEquals(2, fake.count("close"))

        advanceUntil("the second retry", 10000) { fake.count("open") == 3 }
        val second = SystemClock.elapsedRealtime() - secondFailedAt
        assertTrue("second retry after $second ms, expected about 5000", second in 4950..5100)
        assertEquals(State.RUNNING, link.state)
        assertEquals(listOf(FakeEngine.Call("watch", listOf(f2, true))), fake.calls("watch"))
    }

    @Test
    fun unplugWhileWaitingCancelsTheRetry() {
        val d = adapter()
        fake.openResults += "boom"
        plug(d, broadcast = false)
        launch()
        assertEquals(State.WAITING, link.state)
        unplug(d)
        settle()
        assertEquals(State.NO_ADAPTER, link.state)
        advance(10000)
        assertEquals(1, fake.count("open"))
        assertEquals(State.NO_ADAPTER, link.state)
    }

    // ------------------------------------------------------------------------------------------ 5. the radio dies
    private fun assertReopensAndTunesAgain(maxDetectMs: Long) {
        val detect = advanceUntil("the app to close the dead radio", maxDetectMs) { fake.count("close") == 1 }
        assertEquals(State.WAITING, link.state)
        assertTrue(link.message, link.message.startsWith("The radio stopped"))
        fake.freezeBanks = false
        advanceUntil("the reopen", 3000) { fake.count("open") == 2 }
        assertEquals(State.RUNNING, link.state)
        assertEquals(List(2) { FakeEngine.Call("watch", listOf(f2, true)) }, fake.calls("watch"))
        lastDetect = detect
    }
    private var lastDetect = 0L

    @Test
    fun deadRadioIsClosedReopenedAndTunedAgain() {
        running()
        fake.dead = true
        assertReopensAndTunesAgain(maxDetectMs = 500)
    }

    @Test
    fun lostReceiverIsClosedReopenedAndTunedAgain() {
        running()
        fake.rxState = "lost"
        fake.rxError = "bulk transfer: no device"
        assertReopensAndTunesAgain(maxDetectMs = 500)
    }

    @Test
    fun receiverInErrorIsClosedReopenedAndTunedAgain() {
        running()
        fake.rxState = "error"
        assertReopensAndTunesAgain(maxDetectMs = 500)
    }

    @Test
    fun radioClosedUnderTheAppIsReopenedAtOnceNotAfterAStall() {
        running()
        fake.isOpen = false   // closed by someone else: the status says "open": false, and nothing more will come
        assertReopensAndTunesAgain(maxDetectMs = 500)
    }

    @Test
    fun radioThatOpensButNeverStreamsIsRetriedLessOftenAndSaysWhatToDo() {
        running()
        fake.freezeBanks = true
        // open -> no samples for 4 s -> close -> wait -> open ..., with no healthy spell in between
        advanceUntil("the fourth failure in a row", 60000) { fake.count("close") >= UsbLink.PERSISTENT_FAILURES }
        assertEquals(State.WAITING, link.state)
        assertTrue(link.message, link.message.contains("unplug the adapter"))
        val closes = fake.count("close")
        // and it still recovers by itself once the stream comes back
        fake.freezeBanks = false
        advanceUntil("the reopen", UsbLink.SLOW_RETRY_MS + 1000) { link.state == State.RUNNING }
        advance(UsbLink.HEALTHY_MS + 2000)
        assertEquals(State.RUNNING, link.state)
        assertEquals("no more closes once it streams", closes, fake.count("close"))
    }

    @Test
    fun stalledBankCounterIsClosedReopenedAndTunedAgainAfterFourSeconds() {
        running()
        advance(1000)
        fake.freezeBanks = true
        assertReopensAndTunesAgain(maxDetectMs = 6000)
        assertTrue("stall declared after $lastDetect ms, expected just over 4000", lastDetect in 4000..4600)
    }

    @Test
    fun banksFrozenForThreeSecondsThenMovingAgainIsNotAStall() {
        running()
        fake.freezeBanks = true
        advance(3000)
        fake.freezeBanks = false
        advance(10000)
        assertEquals(State.RUNNING, link.state)
        assertEquals(0, fake.count("close"))
        assertEquals(1, fake.count("open"))
    }

    @Test
    fun pauseInPollingIsNotMistakenForAStall() {
        running()
        advance(1000)
        controller.pause().stop()
        settle()
        advance(10000)
        // the first second back the counter has not moved yet (as when nothing was polled meanwhile)
        fake.freezeBanks = true
        controller.restart().resume()
        settle()
        advance(1000)
        fake.freezeBanks = false
        advance(6000)
        assertEquals(State.RUNNING, link.state)
        assertEquals(0, fake.count("close"))
        assertEquals(1, fake.count("open"))
    }

    // ------------------------------------------------------------------------------------------ 6. USB 3 switch
    @Test
    fun usb3SwitchWaitsForTheAdapterToComeBackAndOpensIt() {
        saveChannel(5760, 'F')
        fake.switchResults += "requested"
        val d = adapter()
        plug(d, broadcast = false)
        launch()
        assertEquals(State.SWITCHING, link.state)
        advance(300)
        assertEquals("Switching to USB 3", headline)
        assertEquals(0, fake.count("open"))

        unplug(d)   // re-enumeration: off the bus ...
        settle()
        assertEquals(State.SWITCHING, link.state)
        advance(1500)
        assertEquals(State.SWITCHING, link.state)

        val back = adapter()   // ... and back as a new (SuperSpeed) device, which needs permission again
        plug(back, permission = false)
        assertEquals(State.PERMISSION, link.state)
        answerPermission(true)
        assertEquals(State.RUNNING, link.state)
        assertEquals(1, fake.count("open"))
        assertEquals(2, fake.count("usb3Switch"))
        assertEquals(FakeEngine.Call("watch", listOf(f2, true)), fake.last("watch"))
    }

    @Test
    fun usb3SwitchWhoseAdapterNeverComesBackFailsAfter14SecondsAndTheStatusTapRetries() {
        saveChannel(5760, 'F')
        fake.switchResults += "requested"
        val d = adapter()
        plug(d, broadcast = false)
        launch()
        unplug(d)
        settle()
        advance(13500)
        assertEquals(State.SWITCHING, link.state)
        advance(1000)
        assertEquals(State.FAILED, link.state)
        assertTrue(link.message, link.message.contains("did not come back"))
        assertEquals("Adapter problem", headline)
        assertEquals("Try again", action)

        // it is there after all (no broadcast came): the tap on the status opens it
        plug(adapter(), broadcast = false)
        tapStatus()
        assertEquals(State.RUNNING, link.state)
        assertEquals(1, fake.count("open"))
        assertEquals(FakeEngine.Call("watch", listOf(f2, true)), fake.last("watch"))
    }

    @Test
    fun adapterBackAfterSwitchIsAskedForPermissionOnlyOnce() {
        fake.switchResults += "requested"
        val d = adapter()
        plug(d, broadcast = false)
        launch()
        unplug(d)
        settle()
        // the new device is on the bus a moment before Android's ATTACHED broadcast reaches the app, and the
        // switch watch (once a second) sees it first
        val back = adapter()
        plug(back, permission = false, broadcast = false)
        advance(1000)
        assertEquals(State.PERMISSION, link.state)
        assertEquals(1, permissionRequests.size)
        app.sendBroadcast(android.content.Intent(android.hardware.usb.UsbManager.ACTION_USB_DEVICE_ATTACHED)
            .putExtra(android.hardware.usb.UsbManager.EXTRA_DEVICE, back))
        settle()
        assertEquals("permission requests (one dialog per request)", 1, permissionRequests.size)
    }

    // ------------------------------------------------------------------------------------------ 7. USB 2
    @Test
    fun usb2LinkSaysSoAndTunesNothing() {
        saveChannel(5760, 'F')
        fake.superspeed = false
        plug(adapter(), broadcast = false)
        launch()
        assertEquals(State.USB2, link.state)
        advance(300)
        assertEquals("USB 2 link", headline)
        assertTrue(detail, detail.contains("USB 3"))
        assertEquals("Try again", action)
        assertEquals(0, fake.count("watch"))
        assertEquals(0, fake.count("scan"))
    }

    @Test
    fun usb2TryAgainReopensTheAdapter() {
        fake.superspeed = false
        plug(adapter(), broadcast = false)
        launch()
        assertEquals(State.USB2, link.state)
        advance(300)
        assertEquals("Try again", action)
        fake.superspeed = true
        tapStatus()
        advance(300)
        assertEquals("the tap closes the adapter", 1, fake.count("close"))
        assertEquals("... and opens it again; state ${link.state}, calls ${fake.radioCalls()}", 2, fake.count("open"))
        assertEquals(State.RUNNING, link.state)
    }

    // ------------------------------------------------------------------------------------------ 8. unplug
    @Test
    fun unplugWhileRunningClosesAndPlugInReopensAndTunes() {
        val d = running()
        unplug(d)
        settle()
        assertEquals(State.NO_ADAPTER, link.state)
        assertEquals(1, fake.count("close"))
        advance(300)
        assertEquals("No adapter", headline)

        advance(10000)   // nothing retries on its own with nothing plugged in
        assertEquals(1, fake.count("open"))

        plug(adapter())
        assertEquals(State.RUNNING, link.state)
        assertEquals(2, fake.count("open"))
        assertEquals(List(2) { FakeEngine.Call("watch", listOf(f2, true)) }, fake.calls("watch"))
    }

    @Test
    fun unplugDuringOpeningStandsDownWhenTheOpenReturns() {
        saveChannel(5760, 'F')
        val d = adapter()
        plug(d, broadcast = false)
        launch(holdOpen = true)
        waitUntil("the worker inside open") { fake.insideOpen == 1 }
        assertEquals(State.OPENING, link.state)

        unplug(d)
        assertEquals(State.NO_ADAPTER, link.state)
        fake.release()   // the open in flight now succeeds ...
        waitUntil("the close queued behind it") { fake.count("close") == 1 }
        settle()
        advance(1000)
        assertEquals(State.NO_ADAPTER, link.state)   // ... and must not bring the link up
        assertEquals(0, fake.count("watch"))
        assertEquals(0, fake.count("scan"))
        assertEquals(listOf("open", "close"), fake.radioCalls().map { it.name }.filter { it == "open" || it == "close" })
    }

    @Test
    fun openThatDragsOnSaysSoAfter25Seconds() {
        plug(adapter(), broadcast = false)
        launch(holdOpen = true)
        waitUntil("the worker inside open") { fake.insideOpen == 1 }
        looper.idleFor(Duration.ofMillis(24000))
        assertEquals(State.OPENING, link.state)
        assertEquals("About five seconds.", detail)
        looper.idleFor(Duration.ofMillis(1500))
        assertEquals(State.OPENING, link.state)
        assertTrue(detail, detail.contains("Unplug the adapter"))
        fake.release()
        waitUntil("the open to land") { link.state == State.RUNNING }
    }

    // ------------------------------------------------------------------------------------------ 9. permission
    @Test
    fun permissionDeniedThenAskedAgainThenGranted() {
        val d = adapter()
        plug(d, permission = false, broadcast = false)
        launch()
        assertEquals(State.PERMISSION, link.state)
        assertEquals(1, permissionRequests.size)
        assertEquals(d, permissionRequests[0].first)
        advance(300)
        assertEquals("Waiting for permission", headline)

        answerPermission(false)
        assertEquals(State.DENIED, link.state)
        advance(300)
        assertEquals("Permission needed", headline)
        assertEquals("Ask again", action)
        advance(10000)
        assertEquals("a denial is not asked again by itself", 1, permissionRequests.size)

        tapStatus()
        assertEquals(State.PERMISSION, link.state)
        assertEquals(2, permissionRequests.size)

        answerPermission(true)
        assertEquals(State.RUNNING, link.state)
        assertEquals(1, fake.count("open"))
        assertEquals(FakeEngine.Call("scan", listOf(0.0)), fake.last("scan"))
    }
}
