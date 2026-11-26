package com.fpvdroid.app

import com.fpvdroid.app.FakeEngine.Call
import com.fpvdroid.app.UsbLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

/** The Spectrum tab, against the fake native side: when the radio sweeps, and what the tab shows and tunes. */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35], qualifiers = AppHarness.PHONE, shadows = [RecordingUsbManager::class])
class SpectrumScenariosTest : AppHarness() {
    private val main = 5635e6 to 5955e6   // the span of every band but L
    private val low = 5340e6 to 5640e6

    private fun running(mhz: Int = 5760, band: Char = 'F') {
        saveChannel(mhz, band)
        plug(adapter(), broadcast = false)
        launch()
        assertEquals(State.RUNNING, link.state)
        advance(300)
        assertEquals("watch", fake.mode)
    }

    private fun call(name: String, vararg args: Any?) = Call(name, args.toList())
    private fun sweep(span: Pair<Double, Double>) = call("sweep", span.first, span.second)

    private val graph: SpectrumView get() = views(menu).filterIsInstance<SpectrumView>().single()

    /** The chips under the graph: what the tab says is on the air. */
    private fun used(): List<String> =
        views(graph.parent.parent as android.view.View).filterIsInstance<FoundChip>().map { it.contentDescription.toString() }

    private fun usedChip(name: String): FoundChip =
        views(graph.parent.parent as android.view.View).filterIsInstance<FoundChip>().first { it.contentDescription == name }

    private fun closeTheMenu() {
        click(preview())   // the picture at the rail's foot closes the menu
        advance(300)
    }

    @Suppress("DEPRECATION")
    private fun openTheMenu() {
        activity.onBackPressed()   // from the picture, Back opens the menu
        advance(300)
    }

    // ------------------------------------------------------------------------------------------ when it sweeps
    @Test
    fun openingTheTabSweepsAndLeavingItWatchesTheChannelAgain() {
        running()
        fake.clearCalls()
        showTab("Spectrum")
        assertEquals(listOf(sweep(main)), fake.radioCalls().filter { it.name in setOf("sweep", "watch", "scan", "idle") })
        assertEquals("sweep", fake.mode)
        advance(300)
        assertEquals("Spectrum", headline)
        assertTrue(detail, detail.contains("paused"))
        assertEquals("Sweeping", pill("Sweeping").text.toString())
        assertFalse(pill("Sweeping").isEnabled)

        fake.clearCalls()
        showTab("Channel")
        assertEquals(listOf(call("watch", 5760e6, true)), fake.radioCalls().filter { it.name in setOf("sweep", "watch") })
        assertEquals("watch", fake.mode)
        advance(1000)
        assertEquals(0, fake.count("sweep"))   // and it stays on the channel
    }

    @Test
    fun closingTheMenuEndsTheSweepAndOpeningItAgainDoesNotStartOne() {
        running()
        showTab("Spectrum")
        assertEquals("sweep", fake.mode)
        fake.clearCalls()
        closeTheMenu()
        assertEquals(call("watch", 5760e6, true), fake.last("watch"))
        assertEquals("watch", fake.mode)

        fake.clearCalls()
        openTheMenu()
        assertEquals(Tab.SPECTRUM, menu.tab)   // back where it was
        advance(1000)
        assertEquals(0, fake.count("sweep"))
        assertEquals("watch", fake.mode)
        assertTrue(pill("Sweep").isEnabled)

        click(pill("Sweep"))
        assertEquals(sweep(main), fake.last("sweep"))
        assertEquals("sweep", fake.mode)
    }

    @Test
    fun theLowBandIsSweptOnItsOwn() {
        running()
        showTab("Spectrum")
        assertEquals(sweep(main), fake.last("sweep"))
        val bands = Channels.bands.map { it.first }
        pick(views(menu).filterIsInstance<Segmented>().first { it.isShown && it.label == "Band" }, bands.indexOf('L'), bands.size)
        assertEquals(sweep(low), fake.last("sweep"))
        assertEquals('L', savedSettings().band)
        assertEquals('L', graph.band)
        assertEquals(5340 to 5640, graph.plan.span)
    }

    @Test
    fun aScanUnderWayFinishesBeforeTheSweepStarts() {
        plug(adapter(), broadcast = false)
        launch()   // first run: no channel yet, so it scans
        assertEquals("scan", fake.mode)
        showTab("Spectrum")
        advance(1000)
        assertEquals(0, fake.count("sweep"))
        assertEquals("scan", fake.mode)
        assertFalse(pill("Sweep").isEnabled)

        fake.finishScan(listOf(FakeEngine.Pilot(5769.2, "R4", 31.0)))
        advance(600)
        assertEquals(sweep(main), fake.last("sweep"))
        assertEquals("sweep", fake.mode)
        assertEquals(5769, savedSettings().channelMhz)   // the channel the scan chose is kept for afterwards
    }

    @Test
    fun backFromTheBackgroundTheSweepGoesOn() {
        running()
        showTab("Spectrum")
        controller.pause().stop()
        assertEquals("idle", fake.mode)
        fake.clearCalls()
        controller.start().resume()
        advance(300)
        assertEquals(listOf(sweep(main)), fake.radioCalls().filter { it.name in setOf("sweep", "watch", "scan") })
    }

    // ------------------------------------------------------------------------------------------ what it shows
    @Test
    fun whatIsOnTheAirIsListedByChannelAndPickingOneTunesIt() {
        running()
        fake.carriers += Carrier(5769.2, 12.0, 31f)   // R4
        fake.carriers += Carrier(5712.0, 10.0, 12f)   // between channels: by its frequency
        showTab("Spectrum")
        advance(300)
        assertEquals(listOf("5712", "R4"), used())
        assertTrue(graph.spectrum!!.carriers.size == 2)

        fake.clearCalls()
        click(usedChip("R4"))
        assertEquals(call("watch", 5769e6, true), fake.last("watch"))
        assertEquals("watch", fake.mode)
        val s = savedSettings()
        assertEquals(5769, s.channelMhz)
        assertEquals('R', s.band)
        assertEquals(5769.0, graph.chosenMhz, 0.0)
        advance(1000)
        assertEquals(0, fake.count("sweep"))   // the picture it was looking for: no sweep over it
        assertTrue(pill("Sweep").isEnabled)
        assertEquals(listOf("5712", "R4"), used())   // the last sweep is still on show
    }

    @Test
    fun aTapOnTheGraphPicksTheCarrierUnderItElseTheNearestChannel() {
        running(5658, 'R')
        fake.carriers += Carrier(5806.4, 12.0, 25f)   // R5
        showTab("Spectrum")
        advance(300)
        val g = graph
        assertTrue("the graph is laid out", g.width > 0)
        fun x(mhz: Double) = ((mhz - 5635) / (5955 - 5635) * g.width).toFloat()

        tap(g, x(5810.0), g.height * 0.3f)   // near the carrier: it
        assertEquals(call("watch", 5806e6, true), fake.last("watch"))
        assertEquals(5806, savedSettings().channelMhz)

        tap(g, x(5885.0), g.height * 0.3f)   // nothing on the air near: the nearest R channel (R7, 5880)
        assertEquals(call("watch", 5880e6, true), fake.last("watch"))
        assertEquals(5880, savedSettings().channelMhz)
    }

    // ------------------------------------------------------------------------------------------ digital systems
    private fun segmented(label: String) = views(menu).filterIsInstance<Segmented>().first { it.isShown && it.label == label }

    private fun pickSystem(system: ChannelSystem) = pick(segmented("Channels"), system.ordinal, ChannelSystem.entries.size)

    @Test
    fun djiChannelsAreMarkedAndNamedButNotTuned() {
        running()
        fake.carriers += Carrier(5735.2, 19.0, 28f)   // DJI V1/V2 CH3
        fake.carriers += Carrier(5789.6, 19.0, 25f)   // DJI O4 CH2
        showTab("Spectrum")
        advance(300)
        assertEquals(listOf("B1", "B4"), used())   // as analog channels first

        pickSystem(ChannelSystem.DJI)
        advance(300)
        assertEquals(ChannelSystem.DJI, savedSettings().spectrumSystem)
        assertEquals("DJI V1/V2", graph.plan.title)
        assertEquals(listOf("CH3", "B4"), used())   // off the V1/V2 channels: named as analog
        assertEquals(sweep(main), fake.last("sweep"))   // the same span: the sweep goes on as it was
        assertEquals(1, fake.count("sweep"))

        pick(segmented("DJI"), Plans.djiModels.indexOf("O4"), Plans.djiModels.size)
        advance(300)
        assertEquals("DJI O4", graph.plan.title)
        assertEquals(listOf("B1", "CH2"), used())

        fake.clearCalls()
        click(usedChip("CH2"))   // a DJI channel: there is no picture to be had from it
        assertEquals(0, fake.count("watch"))
        assertEquals("sweep", fake.mode)
        assertTrue(views(menu).any { it is android.widget.TextView && it.isShown && it.text.toString() == "Only analog video can be shown" })
        advance(3500)
        assertFalse(views(menu).any { it is android.widget.TextView && it.isShown && it.text.toString() == "Only analog video can be shown" })

        click(usedChip("B1"))   // off the O4's channels: the analog channel it is on
        assertEquals(call("watch", 5733e6, true), fake.last("watch"))
        assertEquals(ChannelSystem.DJI, savedSettings().spectrumSystem)   // the tab still marks DJI's channels
    }

    @Test
    fun djiSignalsAreNumberedByTheModeTheirWidthSays() {
        running()
        fake.carriers += Carrier(5695.4, 39.0, 22f)   // V1/V2 at 50 Mbps: its CH1, on the 25 Mbps CH2's frequency
        fake.carriers += Carrier(5735.2, 19.5, 28f)   // V1/V2 at 25 Mbps: CH3
        showTab("Spectrum")
        pickSystem(ChannelSystem.DJI)
        advance(300)
        assertEquals(listOf("CH1 50M", "CH3"), used())
        val names = graph.shownNames()
        assertTrue(names.toString(), "CH1 50M" in names && "CH2" !in names)   // the 50 Mbps name in the 25 Mbps one's place
        assertEquals(listOf("CH1", "CH1 50M", "CH3", "CH4", "CH5", "CH8", "CH6", "CH7"), names)

        // narrow again (the pilot went back to 25 Mbps): the 25 Mbps name is back
        fake.carriers[0] = Carrier(5695.4, 19.0, 22f)
        advance(300)
        assertEquals(listOf("CH2", "CH3"), used())
        assertEquals(listOf("CH1", "CH2", "CH3", "CH4", "CH5", "CH8", "CH6", "CH7"), graph.shownNames())

        // O4: the wide channel and CH2 are 5 MHz apart; the width tells them apart where the centre alone would not
        pick(segmented("DJI"), Plans.djiModels.indexOf("O4"), Plans.djiModels.size)
        fake.carriers.clear()
        fake.carriers += Carrier(5792.4, 40.0, 20f)
        advance(300)
        assertEquals(listOf("40/60M"), used())
        fake.carriers[0] = Carrier(5792.4, 19.0, 20f)
        advance(300)
        assertEquals(listOf("CH2"), used())
    }

    /** Off the plan's channels a chip is named by the nearest analog channel, and tapping it tunes that one. */
    @Test
    fun aChipTunesTheChannelItIsNamedBy() {
        running(5658, 'R')
        fake.carriers += Carrier(5802.5, 12.0, 25f)   // 2.5 MHz from F4, 3.5 from R5, on none of the O4's
        showTab("Spectrum")
        pickSystem(ChannelSystem.DJI)
        pick(segmented("DJI"), Plans.djiModels.indexOf("O4"), Plans.djiModels.size)
        advance(300)
        assertEquals(listOf("F4"), used())

        fake.clearCalls()
        click(usedChip("F4"))
        assertEquals(call("watch", 5800e6, true), fake.last("watch"))
        assertEquals('F', savedSettings().band)
        assertEquals(5800, savedSettings().channelMhz)
    }

    @Test
    fun hdzeroChannelsAreMarkedAndChoosingThemMovesTheSweepOffTheLowBand() {
        running(5436, 'L')
        fake.carriers += Carrier(5800.2, 17.0, 24f)   // HDZero F4
        showTab("Spectrum")
        assertEquals(sweep(low), fake.last("sweep"))
        pickSystem(ChannelSystem.HDZERO)
        assertEquals(sweep(main), fake.last("sweep"))
        advance(300)
        assertEquals("HDZero", graph.plan.title)
        assertEquals(listOf("F4"), used())
        assertEquals(listOf("R1", "R2", "E1", "R3", "F1", "F2", "R4", "F4", "R5", "R6", "R7", "R8"), Plans.hdzero.markers.map { it.name })
        assertEquals(0.0, graph.chosenMhz, 0.0)   // the analog channel watched is not one of HDZero's

        fake.clearCalls()
        click(usedChip("F4"))
        assertEquals(0, fake.count("watch"))
        assertEquals('L', savedSettings().band)   // the Channel tab's band is left alone
        assertEquals(5436, savedSettings().channelMhz)
    }

    // ------------------------------------------------------------------------------------------ Wi-Fi
    private fun wifiChips(): List<String> =
        views(graph.parent.parent as android.view.View).filterIsInstance<WifiChip>().map { it.contentDescription.toString() }

    private fun wifiChip(words: String): WifiChip =
        views(graph.parent.parent as android.view.View).filterIsInstance<WifiChip>().first { it.contentDescription == words }

    /** The networks the radio has seen: named on the graph and in chips of their own after the others, never tuned. */
    @Test
    fun wifiNetworksAreNamedButNotTuned() {
        running()
        fake.carriers += Carrier(5769.2, 12.0, 31f)   // R4
        fake.carriers += Carrier(5745.3, 17.5, 20f)   // the network on 149, in use: on the air, on A7's frequency
        fake.wifi += Wifi(5735.0, 5755.0, "Wi-Fi 149", busy = true)
        fake.wifi += Wifi(5855.0, 5895.0, "Wi-Fi 173\u2013177", busy = false)   // 40 MHz, only its beacons heard
        showTab("Spectrum")
        advance(300)
        assertEquals(listOf("A7", "R4"), used())   // the carriers as ever
        assertEquals(listOf("Wi-Fi 149, in use", "Wi-Fi 173\u2013177, idle"), wifiChips())
        assertEquals(listOf("Wi-Fi 149", "Wi-Fi 173\u2013177"), graph.wifi.map { it.label })
        val words = graph.contentDescription.toString()
        assertTrue(words, words.endsWith("In use: A7, R4. Wi-Fi: Wi-Fi 149, in use, Wi-Fi 173\u2013177, idle"))

        fake.clearCalls()
        click(wifiChip("Wi-Fi 173\u2013177, idle"))   // there is no picture to be had from it
        assertEquals(0, fake.count("watch"))
        assertEquals("sweep", fake.mode)
        assertTrue(views(menu).any { it is android.widget.TextView && it.isShown && it.text.toString() == "Only analog video can be shown" })

        // the low band's span holds none of them
        val bands = Channels.bands.map { it.first }
        pick(segmented("Band"), bands.indexOf('L'), bands.size)
        advance(300)
        assertEquals(emptyList<String>(), wifiChips())
        assertTrue(graph.wifi.isEmpty())
    }

    /** The radio's networks as it sends them: the label as it is (an en dash), a broken entry left out, none before. */
    @Test
    fun theRadiosNetworksAreReadAsSent() {
        val sp = Spectrum.parse("""{"first_mhz":5635,"bin_mhz":0.5,"db":[0,null],"carriers":[],"wifi":[""" +
            """{"lo_mhz":5490.0,"hi_mhz":5570.0,"label":"Wi-Fi 100\u2013112","busy":true},{"lo_mhz":5600,"hi_mhz":5580}]}""")
        assertEquals(listOf(Wifi(5490.0, 5570.0, "Wi-Fi 100\u2013112", true)), sp!!.wifi)
        assertEquals(5530.0, sp.wifi[0].mhz, 0.0)
        assertTrue(Spectrum.parse("""{"first_mhz":5635,"bin_mhz":0.5,"db":[0],"carriers":[]}""")!!.wifi.isEmpty())
    }

    @Test
    fun withNoRadioTheTabSaysSo() {
        launch()
        advance(300)
        showTab("Spectrum")
        advance(300)
        assertEquals(0, fake.count("sweep"))
        assertTrue(graph.emptyText, graph.emptyText.contains("radio"))
        assertFalse(pill("Sweep").isEnabled)
    }
}
