package com.kepyur.app

import com.kepyur.app.FakeEngine.Pilot
import com.kepyur.app.UsbLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

/**
 * What a scan leaves, against the fake native side: found signals that are only what the last scan saw, the scan
 * that is owed after one was cut short, and a transmitter off the channel grid.
 */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35], qualifiers = AppHarness.PHONE, shadows = [RecordingUsbManager::class])
class ScanResultScenariosTest : MenuScenarioSteps() {
    // ------------------------------------------------------------------------------------------ stale scan results
    @Test
    fun secondScanThatSeesNothingNewShowsNoChipsSaysSoAndTunesTheNominalFrequency() {
        plug(adapter(), broadcast = false)
        launch()
        assertEquals(listOf(call("scan", 0.0)), fake.calls("scan"))
        advance(300)
        fake.finishScan(listOf(Pilot(5769.2, "R4", 31.0), Pilot(5760.4, "F2", 12.0)))
        advance(300)
        assertEquals(5769, savedSettings().channelMhz)
        assertEquals(setOf("R4", "F2"), chips().toSet())
        assertTrue("bars on R4", tile("R4").bars > 0)
        // while that scan's result stands, the channel is tuned where its transmitter was measured
        layOut()
        fake.clearCalls()
        click(tile("R4"))
        assertEquals(listOf(call("watch", 5769.2e6, true)), fake.calls("watch"))

        advance(10_000)   // the pilots age: the radio last saw them 10 s ago
        fake.clearCalls()
        click(pill("Scan"))
        assertEquals(1, fake.count("scan"))
        advance(1000)
        fake.finishScan(emptyList())   // nothing new
        advance(300)
        assertEquals("no chips after a scan that saw nothing", emptyList<String>(), chips())
        // the radio is back on the saved channel, looking: the status still says what the scan came to
        assertTrue("status should say the scan found no video: '$headline' / '$detail'", detail.contains("found no video"))
        assertEquals("no bars on R4", 0, tile("R4").bars)

        fake.clearCalls()
        layOut()
        click(tile("R4"))
        assertEquals("the nominal frequency, not the old transmitter's", listOf(call("watch", 5769e6, true)), fake.calls("watch"))
    }

    // ------------------------------------------------------------------------------------------ a scan cut short
    @Test
    fun scanCutShortByALongAbsenceRunsWhenTheRadioIsBackAndNotAgainAfterAShortTrip() {
        running()
        click(pill("Scan"))
        assertEquals(listOf(call("scan", f2)), fake.calls("scan"))
        advance(300)
        controller.pause().stop()
        settle()
        advance(MainActivity.BACKGROUND_CLOSE_MS + 1000)
        assertEquals("the adapter was let go", 1, fake.count("close"))

        fake.clearCalls()
        controller.restart().resume()
        waitUntil("the radio up again") { link.state == State.RUNNING }
        settle()
        assertEquals("the owed scan runs", listOf(call("scan", f2)), fake.calls("scan"))
        advance(300)
        fake.finishScan(emptyList())
        advance(300)

        fake.clearCalls()
        controller.pause().stop()
        settle()
        advance(2000)
        controller.restart().resume()
        settle()
        advance(500)
        assertEquals("no scan is owed any more", 0, fake.count("scan"))
        assertEquals(listOf(call("watch", f2, true)), fake.calls("watch"))
    }

    // ------------------------------------------------------------------------------------------ off-grid transmitter
    @Test
    fun offGridTransmitterIsSavedShownAndTunedByItsOwnFrequency() {
        plug(adapter(), broadcast = false)
        launch()
        advance(300)
        fake.finishScan(listOf(Pilot(5653.0, "R1", 30.0)))   // 5 MHz below R1 (5658): on no channel
        advance(300)
        assertEquals(5653, savedSettings().channelMhz)
        assertFalse("R1 is not chosen", tile("R1").chosen)
        assertTrue("no tile is chosen", views(menu).filterIsInstance<ChannelTile>().none { it.chosen })
        assertEquals(listOf("5653"), chips())

        controller.pause().stop().destroy()
        settle()
        fake.clearCalls()
        launch()
        assertEquals(State.RUNNING, link.state)
        assertEquals(listOf(call("watch", 5653e6, true)), fake.calls("watch"))
        assertEquals(0, fake.count("scan"))
    }
}
