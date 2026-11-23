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
