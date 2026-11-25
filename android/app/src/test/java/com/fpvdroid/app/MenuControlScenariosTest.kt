package com.fpvdroid.app

import android.os.SystemClock
import android.view.MotionEvent
import android.view.accessibility.AccessibilityNodeInfo
import android.widget.TextView
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import java.time.Duration

/** The menu's sliders, switches and choices, under a finger and a screen reader. Against the fake native side. */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35], qualifiers = AppHarness.PHONE, shadows = [RecordingUsbManager::class])
class MenuControlScenariosTest : MenuScenarioSteps() {
    // ------------------------------------------------------------------------------------------ sliders
    @Test
    fun sliderTapSetsTheValueAtThatPoint() {
        running()
        viewTab()
        val s = sizeSlider()
        val x = xOf(s, 0.75f)
        val y = midY(s)
        fake.clearCalls()
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x, y, t)
        finger(MotionEvent.ACTION_UP, x, y, t)
        settle()
        assertEquals(0.75f, s.value, 0.02f)
        assertEquals(0.75f, viewOf(fake.last("setView")).scale, 0.02f)
        assertEquals(0.75f, savedSettings().scale, 0.02f)
    }

    @Test
    fun sliderTouchedAndMovedMostlyVerticallyChangesNothing() {
        running()
        viewTab()
        val s = sizeSlider()
        val before = s.value
        val x = xOf(s, 0.75f)
        val y = midY(s)
        fake.clearCalls()
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x, y, t)
        for (i in 1..8) finger(MotionEvent.ACTION_MOVE, x + s.dp(12f) * i / 8, y + s.dp(40f) * i / 8, t)
        finger(MotionEvent.ACTION_UP, x + s.dp(12f), y + s.dp(40f), t)
        settle()
        assertEquals(before, s.value, 0.0001f)
        assertEquals(before, savedSettings().scale, 0.0001f)
        for (c in fake.calls("setView")) assertEquals("setView with another scale: $c", before, viewOf(c).scale, 0.0001f)
    }

    @Test
    fun sliderDraggedSidewaysFollowsTheFingerAndPeeksUntilItLifts() {
        running()
        viewTab()
        val s = sizeSlider()
        val y = midY(s)
        val x0 = xOf(s, 1f)
        val x1 = xOf(s, 1.3f)
        fake.clearCalls()
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x0, y, t)
        val steps = 12
        for (i in 1..steps) {
            finger(MotionEvent.ACTION_MOVE, x0 + (x1 - x0) * i / steps, y, t)
            looper.idle()
        }
        assertEquals("the value follows the finger", 1.3f, s.value, 0.02f)
        assertEquals("while held: the picture sharp", listOf(call("setBackdrop", false, false)), fake.calls("setBackdrop"))
        finger(MotionEvent.ACTION_UP, x1, y, t)
        settle()
        assertEquals(1.3f, s.value, 0.02f)
        assertEquals(1.3f, viewOf(fake.last("setView")).scale, 0.02f)
        assertEquals(1.3f, savedSettings().scale, 0.02f)
        assertEquals(listOf(call("setBackdrop", false, false), call("setBackdrop", true, false)), fake.calls("setBackdrop"))
    }

    /** The preview tapped (its click, as a tap does) while a slider is held: no picture in a preview without a menu. */
    @Test
    fun closeMenuDuringAPeekLeavesThePictureSharpAndTheMenuGone() {
        running()
        viewTab()
        val s = sizeSlider()
        val y = midY(s)
        val x0 = xOf(s, 1f)
        val x1 = xOf(s, 1.2f)
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x0, y, t)
        for (i in 1..8) finger(MotionEvent.ACTION_MOVE, x0 + (x1 - x0) * i / 8, y, t)
        assertEquals(call("setBackdrop", false, false), lastBackdrop())
        fake.clearCalls()
        preview().performClick()
        settle()
        finger(MotionEvent.ACTION_UP, x1, y, t)   // the first finger lifts
        settle()
        fades()
        assertEquals(fake.calls("setBackdrop").toString(), call("setBackdrop", false, false), lastBackdrop())
        assertFalse("the menu is gone", menuShowing)
    }

    /**
     * The same with a real second finger on the preview: the commit makes other fingers go nowhere during a peek, so
     * the menu stays; either way the backdrop must follow the menu.
     */
    @Test
    fun secondFingerOnCloseMenuDuringAPeekIsIgnoredAndTheBackdropFollowsTheMenu() {
        running()
        viewTab()
        val s = sizeSlider()
        val y = midY(s)
        val x0 = xOf(s, 1f)
        val x1 = xOf(s, 1.2f)
        val close = at(preview())
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x0, y, t)
        for (i in 1..8) finger(MotionEvent.ACTION_MOVE, x0 + (x1 - x0) * i / 8, y, t)
        assertEquals(call("setBackdrop", false, false), lastBackdrop())
        twoFingers(pointer1(MotionEvent.ACTION_POINTER_DOWN), t, x1, y, close[0], close[1])
        twoFingers(MotionEvent.ACTION_MOVE, t, x1, y, close[0], close[1])
        twoFingers(pointer1(MotionEvent.ACTION_POINTER_UP), t, x1, y, close[0], close[1])
        settle()
        finger(MotionEvent.ACTION_UP, x1, y, t)
        settle()
        fades()
        assertTrue("the second finger went nowhere: the menu is still there", menuShowing)
        val last = lastBackdrop()!!
        assertEquals("the backdrop follows the menu: $last, calls ${fake.calls("setBackdrop")}", menuShowing, last.args[0])
        assertEquals(1.2f, s.value, 0.02f)
    }

    /** The control for the test above: the same two-finger events, with no peek, do tap the preview (closing the menu). */
    @Test
    fun secondFingerOnCloseMenuWithNothingPeekingClosesIt() {
        running()
        viewTab()
        val s = sizeSlider()
        val y = midY(s)
        val x0 = xOf(s, 1f)
        val close = at(preview())
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x0, y, t)   // resting on the slider, not dragging it
        twoFingers(pointer1(MotionEvent.ACTION_POINTER_DOWN), t, x0, y, close[0], close[1])
        twoFingers(pointer1(MotionEvent.ACTION_POINTER_UP), t, x0, y, close[0], close[1])
        settle()
        finger(MotionEvent.ACTION_UP, x0, y, t)
        settle()
        fades()
        assertFalse("the preview tapped by the second finger closes the menu", menuShowing)
        assertEquals(call("setBackdrop", false, false), lastBackdrop())
    }

    @Test
    fun manualGainTapOnItsOwnValueDoesNotRetuneAndADragRetunesOnceOnLift() {
        running()
        showTab("Signal")
        pick(control("Gain") as Segmented, 1, 2)   // Manual
        assertFalse(savedSettings().gainAuto)
        looper.idleFor(Duration.ofMillis(500))   // the row is scrolled into view
        layOut()
        val g = control("Manual gain") as Slider
        assertTrue(g.isShown)
        assertEquals(30f, g.value, 0.001f)
        val y = midY(g)
        val x30 = xOf(g, 30f)

        fake.clearCalls()
        var t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x30, y, t)
        finger(MotionEvent.ACTION_UP, x30, y, t)
        settle()
        assertEquals("a tap on the value it has: no retune", 0, fake.count("watch"))
        assertEquals(0, fake.count("setGain"))

        val x38 = xOf(g, 37.7f)
        fake.clearCalls()
        t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, x30, y, t)
        for (i in 1..10) {
            finger(MotionEvent.ACTION_MOVE, x30 + (x38 - x30) * i / 10, y, t)
            looper.idle()
        }
        assertEquals("nothing retunes while the finger is down", 0, fake.count("watch"))
        finger(MotionEvent.ACTION_UP, x38, y, t)
        settle()
        assertEquals("once, on lift", listOf(call("watch", f2, false)), fake.calls("watch"))
        assertEquals(call("setGain", 38.0), fake.last("setGain"))
        assertEquals(38f, savedSettings().gainDb, 0.001f)
    }

    // ------------------------------------------------------------------------------------------ switches and choices
    @Test
    fun toggleLiftedFarAwayDoesNotFlip() {
        running()
        viewTab()
        val goggles = control("Goggles") as Toggle
        assertFalse(goggles.isOn)
        val p = at(goggles)
        fake.clearCalls()
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, p[0], p[1], t)
        finger(MotionEvent.ACTION_UP, p[0] - goggles.dp(200f), p[1], t)
        settle()
        assertFalse(goggles.isOn)
        assertFalse(savedSettings().goggles)
        assertEquals(0, fake.count("setView"))
        // the same touch lifted where it landed does flip it (so the one above did reach the switch)
        val t2 = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, p[0], p[1], t2)
        finger(MotionEvent.ACTION_UP, p[0], p[1], t2)
        settle()
        assertTrue(goggles.isOn)
        assertEquals(1, fake.count("setView"))
    }

    @Test
    fun tapOnTheRowTitleFlipsTheSwitch() {
        running()
        viewTab()
        val goggles = control("Goggles") as Toggle
        val title = views(menu).filterIsInstance<TextView>().first { it.text.toString() == "Goggles" }
        fake.clearCalls()
        tapOn(title)
        assertTrue(goggles.isOn)
        assertTrue(savedSettings().goggles)
        assertNotNull("goggles", viewOf(fake.last("setView")).goggles)
    }

    @Test
    fun segmentedActsOnlyWhenLiftedOnTheSegmentItWentDownOn() {
        running()
        viewTab()
        val seg = control("Scaling") as Segmented
        assertEquals(Settings.FIT, seg.selected)
        val y = midY(seg)
        val left = at(seg, 0f, 0f)[0]
        val fitX = left + seg.width * 0.5f / 3
        val cropX = left + seg.width * 2.5f / 3
        fake.clearCalls()
        var t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, cropX, y, t)
        finger(MotionEvent.ACTION_UP, fitX, y, t)
        settle()
        assertEquals("down on Crop, up on Fit: nothing", Settings.FIT, seg.selected)
        assertEquals(0, fake.count("setView"))

        t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, cropX, y, t)
        finger(MotionEvent.ACTION_UP, cropX, y, t)
        settle()
        assertEquals(Settings.CROP, seg.selected)
        assertEquals(1, fake.count("setView"))
        assertEquals(Settings.CROP, viewOf(fake.last("setView")).fit)
        assertEquals(Settings.CROP, savedSettings().fit)
    }

    @Test
    fun segmentedClickWithoutATouchMovesToTheNextChoiceAndRound() {
        running()
        viewTab()
        val seg = control("Scaling") as Segmented
        fake.clearCalls()
        val seen = ArrayList<Int>()
        repeat(3) {
            seg.performClick()
            settle()
            seen += seg.selected
        }
        assertEquals(listOf(Settings.STRETCH, Settings.CROP, Settings.FIT), seen)
        assertEquals(listOf(Settings.STRETCH, Settings.CROP, Settings.FIT), fake.calls("setView").map { viewOf(it).fit })
    }

    // ------------------------------------------------------------------------------------------ accessibility
    @Test
    fun controlsSayWhatTheyAreToAScreenReader() {
        running()
        viewTab()
        val goggles = control("Goggles") as Toggle
        var n = goggles.createAccessibilityNodeInfo()
        assertEquals("android.widget.Switch", n.className.toString())
        assertTrue(n.isCheckable)
        assertFalse(n.isChecked)
        assertEquals("Goggles", n.contentDescription?.toString())
        goggles.performClick()
        settle()
        n = goggles.createAccessibilityNodeInfo()
        assertTrue("checked once on", n.isChecked)

        val seg = control("Scaling") as Segmented
        assertEquals("Scaling: Fit", seg.createAccessibilityNodeInfo().contentDescription?.toString())
        pick(seg, 2, 3)
        assertEquals(Settings.CROP, seg.selected)
        assertEquals("Scaling: Crop", seg.createAccessibilityNodeInfo().contentDescription?.toString())

        val s = sizeSlider()
        val r = s.createAccessibilityNodeInfo().rangeInfo
        assertNotNull("the slider has a range", r)
        assertEquals(0.5f, r.min, 0.0001f)
        assertEquals(1.5f, r.max, 0.0001f)
        assertEquals(1f, r.current, 0.0001f)
        fake.clearCalls()
        assertTrue(s.performAccessibilityAction(AccessibilityNodeInfo.ACTION_SCROLL_FORWARD, null))
        settle()
        assertTrue("raised: ${s.value}", s.value > 1f)
        assertEquals(s.value, viewOf(fake.last("setView")).scale, 0.0001f)
        assertEquals(s.value, s.createAccessibilityNodeInfo().rangeInfo.current, 0.0001f)
    }
}
