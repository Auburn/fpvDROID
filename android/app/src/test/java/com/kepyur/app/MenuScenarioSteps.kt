package com.kepyur.app

import android.hardware.usb.UsbDevice
import android.os.SystemClock
import android.view.InputDevice
import android.view.MotionEvent
import android.view.View
import com.kepyur.app.FakeEngine.Call
import com.kepyur.app.UsbLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.fail
import java.time.Duration

/**
 * What the menu scenarios share: the radio up and watching, the menu closed by its button, and fingers.
 *
 * Touches go in through the activity's decor view, at the window position of what is touched, as a finger's do.
 */
abstract class MenuScenarioSteps : AppHarness() {
    protected val f2 = 5760e6

    /** F2 saved, adapter plugged in with permission, radio up and watching. */
    protected fun running(): UsbDevice {
        saveChannel(5760, 'F')
        val d = adapter()
        plug(d, broadcast = false)
        launch()
        assertEquals(State.RUNNING, link.state)
        advance(300)
        return d
    }

    protected fun call(name: String, vararg args: Any?) = Call(name, args.toList())

    protected val menuShowing get() = menu.visibility == View.VISIBLE

    protected fun closeTheMenu() {
        click(preview())
        looper.idleFor(Duration.ofMillis(300))
        assertFalse("the menu should be closed", menuShowing)
    }

    /** Let the menu's fades finish. */
    protected fun fades() {
        looper.idleFor(Duration.ofMillis(300))
        settle()
    }

    // ------------------------------------------------------------------------------------------ fingers
    protected val decor: View get() = activity.window.decorView

    /** Where ([fx], [fy]) of [v] (fractions of its size) is in the window. */
    protected fun at(v: View, fx: Float = 0.5f, fy: Float = 0.5f): FloatArray {
        val loc = IntArray(2)
        v.getLocationInWindow(loc)
        return floatArrayOf(loc[0] + v.width * fx, loc[1] + v.height * fy)
    }

    protected fun finger(action: Int, x: Float, y: Float, down: Long) = touch(decor, action, x, y, down)

    /** A tap through the window on the middle of [v]. */
    protected fun tapOn(v: View) {
        layOut()
        if (!v.isShown || v.width == 0) fail("$v is not on screen")
        val p = at(v)
        val t = SystemClock.uptimeMillis()
        finger(MotionEvent.ACTION_DOWN, p[0], p[1], t)
        finger(MotionEvent.ACTION_UP, p[0], p[1], t)
        settle()
    }

    /** The window x of [value] on a laid-out slider whose range is [min]..[max]. */
    protected fun xOf(s: Slider, min: Float, max: Float, value: Float): Float {
        val edge = s.dp(12f)
        return at(s, 0f, 0f)[0] + edge + (value - min) / (max - min) * (s.width - 2 * edge)
    }

    protected fun midY(v: View) = at(v)[1]

    /** Two fingers on the window: pointer 0 at ([x0], [y0]), pointer 1 at ([x1], [y1]). */
    protected fun twoFingers(action: Int, down: Long, x0: Float, y0: Float, x1: Float, y1: Float): Boolean {
        val props = Array(2) { i -> MotionEvent.PointerProperties().apply { id = i; toolType = MotionEvent.TOOL_TYPE_FINGER } }
        val coords = arrayOf(
            MotionEvent.PointerCoords().apply { x = x0; y = y0; pressure = 1f; size = 1f },
            MotionEvent.PointerCoords().apply { x = x1; y = y1; pressure = 1f; size = 1f },
        )
        val e = MotionEvent.obtain(down, SystemClock.uptimeMillis(), action, 2, props, coords, 0, 0, 1f, 1f, 0, 0,
            InputDevice.SOURCE_TOUCHSCREEN, 0)
        try {
            return decor.dispatchTouchEvent(e)
        } finally {
            e.recycle()
        }
    }

    protected fun pointer1(action: Int) = action or (1 shl MotionEvent.ACTION_POINTER_INDEX_SHIFT)

    protected fun viewTab() {
        menu.showTab(Tab.VIEW, animate = false)
        looper.idle()
        layOut()
    }

    protected fun sizeSlider() = control("Size") as Slider

    protected fun scaleOf(c: Call?): Float {
        if (c == null || c.name != "setView") fail("expected setView, got $c")
        return c!!.args[2] as Float
    }

    protected fun lastBackdrop(): Call? = fake.last("setBackdrop")

    protected fun chips(): List<String> = views(menu).filterIsInstance<FoundChip>().map { it.contentDescription.toString() }
}
