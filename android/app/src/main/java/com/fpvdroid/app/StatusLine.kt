package com.fpvdroid.app

import android.annotation.SuppressLint
import android.content.Context
import android.text.TextUtils
import android.view.Gravity
import android.widget.LinearLayout

/**
 * The line of the status's words along the top of the menu, the whole width: the detail, and at its end the action
 * ("Try again": a tap on the line does it), or the app's name when there are neither ([MenuState.quiet]). With the
 * menu closed it stays in its place, over the picture, while there is something to say there
 * ([MenuState.sayOverPicture]); a tap on it is then a tap on the picture, but for the action.
 */
@SuppressLint("ViewConstructor")
internal class StatusLine(ctx: Context, onAction: () -> Unit) : LinearLayout(ctx) {
    private val wordmark = label(ctx, "fpvDROID", 11f, Skin.TEXT_3, medium = true).apply { letterSpacing = 0.32f }
    val detail = label(ctx, "", 12.5f, Skin.TEXT_2).overPicture().apply {
        maxLines = 1   // a long message in full: the Signal tab
        ellipsize = TextUtils.TruncateAt.END
    }
    val action = label(ctx, "", 12.5f, Skin.ACCENT, medium = true).overPicture()
    private var hasAction = false
    private var shown = true   // showing, or coming

    init {
        orientation = HORIZONTAL
        gravity = Gravity.TOP
        addView(wordmark, LayoutParams(-2, -2))
        addView(detail, LayoutParams(0, -2, 1f))
        addView(action, LayoutParams(-2, -2).apply { marginStart = dpi(14f) })
        setOnClickListener { if (hasAction) onAction() }
    }

    /** [st]'s words: all of them while the menu is open, else only those worth saying over the picture. */
    fun render(st: MenuState, menuOpen: Boolean) {
        val want = menuOpen || st.sayOverPicture
        // while it is not wanted its words are left as they were (they may be fading)
        if (want) {
            // nothing to say: the name of the app in its place (an action alone keeps its place at the end)
            wordmark.visibility = if (st.quiet) VISIBLE else GONE
            if (detail.text.toString() != st.detail) detail.text = st.detail
            detail.visibility = if (st.quiet) GONE else if (st.detail.isEmpty()) INVISIBLE else VISIBLE
            action.text = st.action ?: ""
            action.visibility = if (st.action == null) GONE else VISIBLE
        }
        hasAction = st.action != null
        isClickable = want && hasAction   // else a tap here is the menu's, or the picture's
        show(want)
    }

    private fun show(want: Boolean) {
        if (want == shown) return
        shown = want
        animate().cancel()
        if (want) visibility = VISIBLE
        animate().alpha(if (want) 1f else 0f).setDuration(FADE_MS)
            .withEndAction(if (want) null else Runnable { visibility = INVISIBLE }).start()
    }

    companion object {
        /** In or out as quickly as the rest of the menu closes, so that on a close the two go together. */
        const val FADE_MS = MenuView.CLOSE_MS
    }
}
