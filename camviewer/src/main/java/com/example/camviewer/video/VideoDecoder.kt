package com.example.camviewer.video

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.util.Log
import android.view.Surface
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.nio.ByteBuffer
import javax.inject.Inject

private const val TAG = "VideoDecoder"

/**
 * H.264/H.265 video decoder using Android MediaCodec.
 * Decodes NAL units in Annex-B format and renders to a Surface.
 */
class VideoDecoder @Inject constructor() {
    
    private var decoder: MediaCodec? = null
    private var inputBufferIndex = -1
    private var frameCount = 0
    
    /**
     * Initialize decoder with codec type and output surface
     * @param codecType "video/avc" for H.264 or "video/hevc" for H.265
     * @param width Video width
     * @param height Video height
     * @param surface Output surface for rendering
     */
    suspend fun initialize(
        codecType: String = MediaFormat.MIMETYPE_VIDEO_HEVC, // H.265 default
        width: Int = 1920,
        height: Int = 1080,
        surface: Surface?
    ) = withContext(Dispatchers.Default) {
        release()
        
        Log.i(TAG, "Initializing MediaCodec: $codecType ${width}x${height}, surface: ${surface != null}")
        
        try {
            decoder = MediaCodec.createDecoderByType(codecType).apply {
                val format = MediaFormat.createVideoFormat(codecType, width, height).apply {
                    // Request low latency decoding
                    setInteger(MediaFormat.KEY_LOW_LATENCY, 1)
                    
                    // Use surface rendering (hardware acceleration)
                    if (surface != null) {
                        setInteger(MediaFormat.KEY_COLOR_FORMAT,
                            MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
                    }
                }
                
                configure(format, surface, null, 0)
                start()
            }
            
            frameCount = 0
            Log.i(TAG, "MediaCodec started successfully")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize MediaCodec", e)
            decoder = null
            throw e
        }
    }
    
    /**
     * Decode a video frame (NAL units in Annex-B format)
     * @param data Frame data containing one or more NAL units with start codes
     * @param timestampUs Presentation timestamp in microseconds
     * @return true if frame was successfully queued for decoding
     */
    suspend fun decodeFrame(data: ByteArray, timestampUs: Long): Boolean = withContext(Dispatchers.Default) {
        val codec = decoder ?: return@withContext false
        
        try {
            // Get input buffer
            inputBufferIndex = codec.dequeueInputBuffer(10_000) // 10ms timeout
            
            if (inputBufferIndex >= 0) {
                val inputBuffer = codec.getInputBuffer(inputBufferIndex)
                inputBuffer?.let { buffer ->
                    buffer.clear()
                    buffer.put(data)
                    
                    codec.queueInputBuffer(
                        inputBufferIndex,
                        0,
                        data.size,
                        timestampUs,
                        0
                    )
                    
                    frameCount++
                    
                    if (frameCount <= 5 || frameCount % 100 == 0) {
                        Log.d(TAG, "Queued frame $frameCount: ${data.size} bytes")
                    }
                }
                
                // Release output buffers (rendering happens automatically to surface)
                releaseOutputBuffers(codec)
                
                return@withContext true
            } else {
                if (frameCount <= 5) {
                    Log.w(TAG, "No input buffer available (timeout)")
                }
            }
            
            return@withContext false
        } catch (e: Exception) {
            Log.e(TAG, "Error decoding frame", e)
            return@withContext false
        }
    }
    
    /**
     * Release any available output buffers
     */
    private fun releaseOutputBuffers(codec: MediaCodec) {
        val info = MediaCodec.BufferInfo()
        var outputCount = 0
        
        // Process all available output buffers
        var outputBufferIndex = codec.dequeueOutputBuffer(info, 0)
        while (outputBufferIndex >= 0) {
            // Release buffer to render to surface
            codec.releaseOutputBuffer(outputBufferIndex, true)
            outputCount++
            
            outputBufferIndex = codec.dequeueOutputBuffer(info, 0)
        }
        
        if (outputCount > 0 && frameCount <= 10) {
            Log.d(TAG, "Released $outputCount output buffers to surface")
        }
        
        // Handle format changes
        if (outputBufferIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
            val newFormat = codec.outputFormat
            Log.i(TAG, "Output format changed: $newFormat")
        }
    }
    
    /**
     * Flush decoder buffers (call when seeking or resetting)
     */
    suspend fun flush() = withContext(Dispatchers.Default) {
        try {
            decoder?.flush()
        } catch (e: Exception) {
            // Ignore
        }
    }
    
    /**
     * Release decoder resources
     */
    suspend fun release() = withContext(Dispatchers.Default) {
        try {
            decoder?.stop()
            decoder?.release()
        } catch (e: Exception) {
            // Ignore
        } finally {
            decoder = null
            inputBufferIndex = -1
            frameCount = 0
        }
    }
    
    /**
     * Get total decoded frame count
     */
    fun getFrameCount(): Int = frameCount
}
