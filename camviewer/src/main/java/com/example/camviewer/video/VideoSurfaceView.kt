package com.example.camviewer.video

import android.content.Context
import android.util.AttributeSet
import android.view.SurfaceHolder
import android.view.SurfaceView

/**
 * Custom SurfaceView for video rendering.
 * Provides a Surface for MediaCodec to render decoded frames directly.
 */
class VideoSurfaceView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : SurfaceView(context, attrs, defStyleAttr), SurfaceHolder.Callback {
    
    private var surfaceReadyCallback: ((SurfaceHolder) -> Unit)? = null
    private var surfaceDestroyedCallback: (() -> Unit)? = null
    
    init {
        holder.addCallback(this)
    }
    
    /**
     * Set callback for when surface is ready
     */
    fun setOnSurfaceReadyListener(callback: (SurfaceHolder) -> Unit) {
        surfaceReadyCallback = callback
    }
    
    /**
     * Set callback for when surface is destroyed
     */
    fun setOnSurfaceDestroyedListener(callback: () -> Unit) {
        surfaceDestroyedCallback = callback
    }
    
    override fun surfaceCreated(holder: SurfaceHolder) {
        surfaceReadyCallback?.invoke(holder)
    }
    
    override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
        // Surface size changed - could adjust decoder if needed
    }
    
    override fun surfaceDestroyed(holder: SurfaceHolder) {
        surfaceDestroyedCallback?.invoke()
    }
}
