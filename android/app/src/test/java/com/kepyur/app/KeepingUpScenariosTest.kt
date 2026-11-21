package com.kepyur.app

import android.widget.TextView
import com.kepyur.app.FakeEngine.Pilot
import com.kepyur.app.UsbLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import java.time.Duration

/**
 * Is the phone keeping up: the blocks lost on the way, as counted on the Signal tab, and when they bring up the
 * advice. Against the fake native side.
 */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35], qualifiers = AppHarness.PHONE, shadows = [RecordingUsbManager::class])
class KeepingUpScenariosTest : MenuScenarioSteps() {
    // ------------------------------------------------------------------------------------------ USB link drops
    private fun statsText(): String =
        (MenuView::class.java.getDeclaredField("stats").apply { isAccessible = true }.get(menu) as TextView).text.toString()

    @Test
    fun usbStreamDropsAreCountedAndSayThePhoneIsNotKeepingUp() {
        running()
        advance(500)
        assertTrue(statsText(), statsText().contains(" 0 dropped"))
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)

        fake.dropped = 2
        fake.streamDropped = 5
        advance(500)
        assertTrue("the numbers count the USB link's drops too: ${statsText()}", statsText().contains(" 7 dropped"))
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)

        fake.dropped = 0
        fake.streamDropped = 5
        advance(500)
        // stream drops alone, rising
        fake.streamDropped = 9
        advance(500)
        assertTrue(statsText(), statsText().contains(" 9 dropped"))
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)
        advance(6000)   // no more drops: the warning goes after 5 s
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        assertEquals(State.RUNNING, link.state)
    }

    @Test
    fun oneLateUsbBlockNowAndThenIsNotThePhoneFailingToKeepUp() {
        running()
        advance(6000)
        // one block the adapter's stream gave up on, then another well apart: any phone stalls for a moment
        fake.streamDropped = 1
        advance(500)
        assertTrue("it is counted: ${statsText()}", statsText().contains(" 1 dropped"))
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        advance(8000)
        fake.streamDropped = 2
        advance(500)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)

        // several close together are another matter
        advance(8000)
        fake.streamDropped = 5
        advance(500)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)

        // and one block lost in the queue behind the link always is: that is the phone's own processing
        advance(8000)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.dropped = 1
        advance(500)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)
    }

    @Test
    fun threeLateUsbBlocksCountTogetherOnlyWithinTheWindow() {
        running()
        advance(6000)
        // three seen within about four seconds: the link is not being served in time
        fake.streamDropped = 1; advance(2000)
        fake.streamDropped = 2; advance(2000)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.streamDropped = 3; advance(500)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)

        // the same three spread over six seconds are not
        advance(10_000)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.streamDropped = 4; advance(3000)
        fake.streamDropped = 5; advance(3000)
        fake.streamDropped = 6; advance(500)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)

        // and one every three seconds, for a minute, never is
        advance(10_000)
        for (i in 7L..26L) {
            fake.streamDropped = i; advance(3000)
            assertNotEquals("at $i", MainActivity.OVERLOAD_ADVICE, detail)
        }
    }

    @Test
    fun lateUsbBlocksFromTheBackgroundAreNotCountedOnReturn() {
        running()
        advance(6000)
        assertEquals("watch", fake.mode)
        // in the background nothing is refreshed, and the adapter streams on for a while
        controller.pause().stop()
        settle()
        advance(5000); fake.streamDropped = 1
        advance(8000); fake.streamDropped = 2
        advance(8000); fake.streamDropped = 3
        advance(2000)
        controller.restart().resume()
        settle()
        advance(300)
        assertEquals("watch", fake.mode)
        assertNotEquals("three late blocks in 23 s unseen", MainActivity.OVERLOAD_ADVICE, detail)

        // back on screen, the count goes on from where it now stands
        advance(3000)
        fake.streamDropped = 4; advance(500)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.streamDropped = 6; advance(500)
        assertEquals("three seen within the window", MainActivity.OVERLOAD_ADVICE, detail)
    }

    @Test
    fun lateUsbBlocksOfAScanAreNotHeldAgainstTheWatchAfterIt() {
        running()
        advance(6000)
        showTab("Channel")
        click(pill("Scan"))
        settle()
        assertEquals("scan", fake.mode)
        advance(1000)
        fake.streamDropped = 2   // a scan retunes all the time: late blocks there are its own
        advance(1000)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.finishScan(listOf(FakeEngine.Pilot(5760.0, "F2", 31.0)))
        advance(600)
        assertEquals("watch", fake.mode)
        fake.streamDropped = 3   // the first one of the watch
        advance(500)
        assertNotEquals("one late block in this watch", MainActivity.OVERLOAD_ADVICE, detail)
    }

    /** The same for the scan's last moments: blocks that no refresh saw while it was still scanning. */
    @Test
    fun lateUsbBlocksOfAScansLastMomentsAreNotHeldAgainstTheWatchAfterIt() {
        running()
        advance(6000)
        showTab("Channel")
        click(pill("Scan"))
        settle()
        assertEquals("scan", fake.mode)
        advance(1000)
        fake.streamDropped = 2   // after the screen's last look at the scan
        fake.finishScan(listOf(FakeEngine.Pilot(5760.0, "F2", 31.0)))
        advance(600)
        assertEquals("watch", fake.mode)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.streamDropped = 3   // the first one of the watch
        advance(500)
        assertNotEquals("one late block in this watch", MainActivity.OVERLOAD_ADVICE, detail)
        fake.streamDropped = 6   // and the watch's own are still counted
        advance(500)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)
    }

    /**
     * And for a stream test: the refresh that sees the last test done starts the watch itself, in the same pass.
     * The radio was not watching at that refresh, so what comes in before the next one is not the watch's.
     */
    @Test
    fun lateUsbBlocksOfTheRetuneAfterAStreamTestAreNotHeldAgainstTheWatch() {
        saveChannel(5760, 'F')
        plug(adapter(), broadcast = false)
        launch(android.content.Intent(app, MainActivity::class.java).putExtra("mode", "usbtest"))
        advance(500)
        fake.finishStreamTest(40.0)
        advance(2500)
        fake.finishStreamTest(20.0)
        toNextPoll()   // reads idle with the test done: the next test, or none left and the watch
        assertEquals("watch", fake.mode)
        fake.streamDropped = 3   // the retune into the watch
        toNextPoll()
        advance(300)
        assertNotEquals("blocks from before the watch's first refresh", MainActivity.OVERLOAD_ADVICE, detail)
        fake.streamDropped = 6   // the watch's own
        advance(500)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)
    }

    /** One ms at a time until the status has been read once more (a refresh). */
    private fun toNextPoll() {
        val n = fake.statusCalls
        var steps = 0
        while (fake.statusCalls == n) {
            looper.idleFor(Duration.ofMillis(1))
            settle()
            if (++steps > 2000) fail("no status poll in 2 s")
        }
    }
}
