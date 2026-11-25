package com.fpvdroid.app

import android.widget.TextView
import com.fpvdroid.app.UsbLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

/**
 * Is the phone keeping up: the blocks lost on the way, as counted on the Signal tab, and when they bring up the
 * advice. Against the fake native side. Which late blocks belong to the watch on screen, and for how long they
 * count, is the radio's to say ("late_recent" in its status: native/tests/radio_tests.cpp).
 */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35], qualifiers = AppHarness.PHONE, shadows = [RecordingUsbManager::class])
class KeepingUpScenariosTest : MenuScenarioSteps() {
    private fun statsText(): String =
        (MenuView::class.java.getDeclaredField("stats").apply { isAccessible = true }.get(menu) as TextView).text.toString()

    @Test
    fun blocksLostAnywhereOnTheWayAreCountedOnTheSignalTab() {
        running()
        advance(500)
        assertTrue(statsText(), statsText().contains(" 0 dropped"))
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)

        fake.dropped = 2
        fake.streamDropped = 5
        advance(500)
        assertTrue("the numbers count the USB link's drops too: ${statsText()}", statsText().contains(" 7 dropped"))
    }

    /** The picture's delay, while a picture is locked: as far along the way to the screen as it was measured. */
    @Test
    fun thePicturesDelayIsOnTheSignalTab() {
        running()
        advance(500)
        assertTrue("nothing measured, nothing said: ${statsText()}", !statsText().contains("Picture delay"))

        fun measured(json: String): String {
            fake.latencyJson = json
            advance(500)
            return statsText().lines().firstOrNull { it.startsWith("Picture delay") } ?: ""
        }
        fake.locked = true
        // the display says when it showed each frame
        assertEquals("Picture delay 46 ms at the top, 31 at the bottom · 120 Hz display", measured(
            """{"queued_top_ms":24.0,"queued_bottom_ms":9.0,"shown_top_ms":46.2,"shown_bottom_ms":31.0,""" +
                """"expected_top_ms":45.0,"expected_bottom_ms":30.0,"refresh_hz":120.0,"front":false}"""))
        // it does not: as the compositor expected to
        assertEquals("Picture delay 37 ms at the top, 33 at the bottom · 60 Hz display", measured(
            """{"queued_top_ms":4.0,"queued_bottom_ms":2.5,"shown_top_ms":0.0,"shown_bottom_ms":0.0,""" +
                """"expected_top_ms":37.0,"expected_bottom_ms":33.0,"refresh_hz":60.0,"front":false}"""))
        // front-buffer drawing
        assertEquals("Picture delay 9 ms at the top, 7 at the bottom, drawn straight to the screen · 120 Hz display", measured(
            """{"queued_top_ms":4.7,"queued_bottom_ms":2.8,"expected_top_ms":8.9,"expected_bottom_ms":7.0,""" +
                """"refresh_hz":120.0,"front":true}"""))
        // only up to the hand-over, and the display's rate not known
        assertEquals("Picture delay 24 ms at the top, 9 at the bottom, before the display",
            measured("""{"queued_top_ms":24.0,"queued_bottom_ms":9.0,"refresh_hz":0.0}"""))
        assertEquals("not JSON: nothing said", "", measured("{"))
        fake.locked = false
        assertEquals("no picture: nothing said", "", measured("""{"shown_top_ms":46.0,"shown_bottom_ms":31.0}"""))
    }

    @Test
    fun aBlockLostInTheQueueSaysThePhoneIsNotKeepingUpForFiveSeconds() {
        running()
        advance(6000)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.dropped = 1   // behind the link: that is the phone's own processing, and one is enough
        advance(500)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)
        advance(6000)   // no more of them: the warning goes after 5 s
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        assertEquals(State.RUNNING, link.state)
    }

    @Test
    fun lateUsbBlocksSayItOnlyWhileTheRadioCountsEnoughOfThemInThisWatch() {
        running()
        advance(6000)
        // one block the adapter's stream gave up on, now and then: any phone stalls for a moment
        fake.streamDropped = 1; fake.lateRecent = 1
        advance(500)
        assertTrue("it is counted: ${statsText()}", statsText().contains(" 1 dropped"))
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.streamDropped = 2; fake.lateRecent = 2
        advance(500)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)

        // several close together are another matter, for as long as the radio's window holds them
        fake.streamDropped = 3; fake.lateRecent = MainActivity.LATE_BLOCKS.toLong()
        advance(500)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)
        advance(3000)
        assertEquals(MainActivity.OVERLOAD_ADVICE, detail)
        fake.lateRecent = 1   // the others have left the window
        advance(500)
        assertNotEquals(MainActivity.OVERLOAD_ADVICE, detail)
        assertTrue("still counted: ${statsText()}", statsText().contains(" 3 dropped"))
    }
}
