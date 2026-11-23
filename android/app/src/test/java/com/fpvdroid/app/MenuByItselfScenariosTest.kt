package com.fpvdroid.app

import android.widget.TextView
import com.fpvdroid.app.UsbLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

/**
 * The menu that opens by itself when the picture goes with nothing on screen to say why (the adapter gone, a
 * radio being started again), and closes by itself when the picture is back. Against the fake native side.
 */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35], qualifiers = AppHarness.PHONE, shadows = [RecordingUsbManager::class])
class MenuByItselfScenariosTest : MenuScenarioSteps() {
    // ------------------------------------------------------------------------------------------ the menu by itself
    @Test
    fun menuClosedUnplugOpensItByItselfAndReplugClosesItAgainAndWatches() {
        val d = running()
        closeTheMenu()
        fake.clearCalls()
        unplug(d)
        settle()
        assertEquals(State.NO_ADAPTER, link.state)
        assertTrue("the menu shows itself when the picture goes", menuShowing)
        assertTrue(fake.calls("setBackdrop").toString(), call("setBackdrop", true, false) in fake.calls("setBackdrop"))
        fades()
        assertEquals("No adapter", headline)

        fake.clearCalls()
        plug(adapter())
        assertEquals(State.RUNNING, link.state)
        fades()
        assertFalse("the menu steps aside again once the radio is up", menuShowing)
        assertEquals(call("setBackdrop", false, false), lastBackdrop())
        assertEquals(listOf(call("watch", f2, true)), fake.calls("watch"))
    }

    @Test
    fun menuThatOpenedItselfAndWasTouchedStaysOpenWhenTheRadioIsBack() {
        val d = running()
        closeTheMenu()
        unplug(d)
        settle()
        assertTrue(menuShowing)
        fades()
        // a finger on the status's words along the top (no action in NO_ADAPTER: harmless)
        val status = views(menu).filterIsInstance<TextView>().first { it.text.toString().startsWith("Plug the Wi-Fi adapter") }
        tapOn(status)
        assertEquals(State.NO_ADAPTER, link.state)

        fake.clearCalls()
        plug(adapter())
        assertEquals(State.RUNNING, link.state)
        fades()
        assertTrue("touched since it opened: it stays", menuShowing)
        assertFalse(fake.calls("setBackdrop").toString(), call("setBackdrop", false, false) in fake.calls("setBackdrop"))
        assertEquals(listOf(call("watch", f2, true)), fake.calls("watch"))
    }

    @Test
    fun radioDyingWithTheMenuClosedOpensItAndTheReopenClosesIt() {
        running()
        closeTheMenu()
        fake.dead = true
        advanceUntil("the app to close the dead radio", 1000) { fake.count("close") == 1 }
        assertEquals(State.WAITING, link.state)
        assertTrue("the menu shows itself while the radio is started again", menuShowing)
        advanceUntil("the reopen", 4000) { link.state == State.RUNNING }
        fades()
        assertFalse("and closes again once it is back", menuShowing)
        assertEquals(call("setBackdrop", false, false), lastBackdrop())
        assertEquals(List(2) { call("watch", f2, true) }, fake.calls("watch"))
    }

    @Test
    fun menuOpenedByTheUserIsNotClosedByAnUnplugAndReplug() {
        val d = running()
        assertTrue(menuShowing)   // open after launch, as always
        unplug(d)
        settle()
        assertTrue(menuShowing)
        plug(adapter())
        assertEquals(State.RUNNING, link.state)
        fades()
        assertTrue("it was not opened by itself: it stays", menuShowing)
        assertEquals(call("setBackdrop", true, true), lastBackdrop())   // only the launch's
    }

    @Test
    fun adapterGoneDuringAShortAbsenceShowsTheMenuOnReturn() {
        val d = running()
        closeTheMenu()
        controller.pause().stop()
        settle()
        unplug(d)
        settle()
        assertEquals(State.NO_ADAPTER, link.state)
        advance(2000)
        controller.restart().resume()
        settle()
        fades()
        assertTrue("no picture and nothing to say why: the menu should show 'No adapter'", menuShowing)
    }

    // ------------------------------------------------------------------------------------------ long absence
    @Test
    fun longAbsenceShowsTheMenuWhileTheAdapterOpensAndClosesItOnceTheRadioIsUp() {
        running()
        closeTheMenu()
        controller.pause().stop()
        settle()
        advance(MainActivity.BACKGROUND_CLOSE_MS + 1000)
        assertEquals("the adapter was let go", 1, fake.count("close"))

        fake.clearCalls()
        fake.hold()
        controller.restart().resume()
        waitUntil("the worker inside open") { fake.insideOpen == 1 }
        assertEquals(State.OPENING, link.state)
        assertTrue("the menu says why there is no picture yet", menuShowing)
        assertTrue(fake.calls("setBackdrop").toString(), call("setBackdrop", true, false) in fake.calls("setBackdrop"))

        fake.release()
        waitUntil("the radio up again") { link.state == State.RUNNING }
        settle()
        fades()
        assertFalse("closed by itself once the radio is up", menuShowing)
        assertEquals(call("setBackdrop", false, false), lastBackdrop())
        assertEquals(listOf(call("watch", f2, true)), fake.calls("watch"))
    }

    @Test
    fun shortAbsenceDoesNotOpenTheMenu() {
        running()
        closeTheMenu()
        controller.pause().stop()
        settle()
        advance(2000)
        fake.clearCalls()
        controller.restart().resume()
        settle()
        fades()
        assertFalse(menuShowing)
        assertEquals(0, fake.count("setBackdrop"))
        assertEquals(listOf(call("watch", f2, true)), fake.calls("watch"))
    }
}
