package com.example.camviewer.video

import android.content.Context
import android.media.MediaCodec
import android.media.MediaFormat
import android.media.MediaMuxer
import android.os.Environment
import android.util.Log
import dagger.hilt.android.qualifiers.ApplicationContext
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import java.io.File
import java.nio.ByteBuffer
import java.text.SimpleDateFormat
import java.util.*
import javax.inject.Inject

private const val TAG = "VideoRecorder"

/**
 * Records H.264/H.265 video stream to MP4 file
 */
class VideoRecorder @Inject constructor(
    @ApplicationContext private val context: Context
) {
    private val mutex = Mutex()
    private var muxer: MediaMuxer? = null
    private var trackIndex: Int = -1
    private var currentFile: File? = null
    private var isRecording = false
    private var bytesWritten = 0L
    private var muxerStarted = false
    private var currentCodec: String = "h265"
    private var startTimeUs: Long = 0L
    
    /**
     * Start recording to a new MP4 file
     * Files are saved to Movies/recomoLive/ directory
     * @return File path if successful, null otherwise
     */
    suspend fun startRecording(codec: String = "h265"): String? = withContext(Dispatchers.IO) {
        if (isRecording) {
            Log.w(TAG, "Already recording")
            return@withContext null
        }
        
        try {
            currentCodec = codec
            
            // Use public Movies directory for easy access
            val outputDir = File("/sdcard/Movies/recomoVideosRawStream").apply {
                if (!exists()) mkdirs()
            }
            
            // Generate filename with timestamp
            val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
            currentFile = File(outputDir, "video_${timestamp}.mp4")
            
            // Create MediaMuxer for MP4
            muxer = MediaMuxer(currentFile!!.absolutePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
            
            isRecording = true
            muxerStarted = false
            bytesWritten = 0L
            trackIndex = -1
            
            Log.i(TAG, "Started recording to: ${currentFile!!.absolutePath}")
            currentFile!!.absolutePath
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start recording", e)
            cleanup()
            null
        }
    }
    
    /**
     * Write video frame data to MP4
     * @param data H.264/H.265 NAL unit in Annex-B format
     */
    suspend fun writeFrame(data: ByteArray) = withContext(Dispatchers.IO) {
        mutex.withLock {
            if (!isRecording) {
                Log.d(TAG, "writeFrame called but not recording")
                return@withLock
            }
            if (muxer == null) {
                Log.e(TAG, "writeFrame called but muxer is null")
                return@withLock
            }
            
            Log.v(TAG, "writeFrame: ${data.size} bytes, muxerStarted=$muxerStarted")
            
            try {
                // On first frame, setup the muxer with format
                if (!muxerStarted) {
                    // CRITICAL: Wait for keyframe before starting muxer
                    // Only keyframes contain VPS/SPS/PPS needed for CSD
                    if (!isKeyFrame(data)) {
                        Log.d(TAG, "Buffering frame, waiting for keyframe (${data.size} bytes)")
                        return@withLock
                    }
                    
                    Log.d(TAG, "Setting up muxer with keyframe (${data.size} bytes)")
                    val format = extractMediaFormat(data, currentCodec)
                    if (format != null) {
                        trackIndex = muxer!!.addTrack(format)
                        muxer!!.start()
                        muxerStarted = true
                        startTimeUs = System.nanoTime() / 1000
                        Log.i(TAG, "MediaMuxer started with track index: $trackIndex")
                    } else {
                        Log.e(TAG, "Failed to create format, cannot start muxer")
                        return@withLock
                    }
                }
                
                if (muxerStarted && trackIndex >= 0) {
                    // MediaMuxer expects raw NAL units without start codes
                    // Strip Annex-B start codes (0x00 0x00 0x00 0x01 or 0x00 0x00 0x01)
                    var offset = 0
                    if (data.size >= 4 && data[0] == 0.toByte() && data[1] == 0.toByte()) {
                        if (data[2] == 0.toByte() && data[3] == 1.toByte()) {
                            offset = 4 // 4-byte start code
                        } else if (data[2] == 1.toByte()) {
                            offset = 3 // 3-byte start code
                        }
                    }
                    
                    // Write frame data with relative timestamp
                    val presentationTimeUs = (System.nanoTime() / 1000) - startTimeUs
                    
                    val buffer = ByteBuffer.wrap(data, offset, data.size - offset)
                    val bufferInfo = MediaCodec.BufferInfo()
                    bufferInfo.offset = 0
                    bufferInfo.size = data.size - offset
                    bufferInfo.presentationTimeUs = presentationTimeUs
                    bufferInfo.flags = if (isKeyFrame(data)) MediaCodec.BUFFER_FLAG_KEY_FRAME else 0
                    
                    muxer!!.writeSampleData(trackIndex, buffer, bufferInfo)
                    bytesWritten += data.size.toLong()
                    
                    if (bytesWritten % (100 * 1024) == 0L) { // Log every 100KB instead of 1MB
                        Log.d(TAG, "Written ${bytesWritten / 1024} KB")
                    }
                } else {
                    Log.e(TAG, "Cannot write frame: muxerStarted=$muxerStarted, trackIndex=$trackIndex")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Failed to write frame", e)
            }
        }
    }
    
    /**
     * Extract MediaFormat from H.264/H.265 stream
     */
    private fun extractMediaFormat(data: ByteArray, codec: String): MediaFormat? {
        try {
            val mime = if (codec.contains("265") || codec.contains("hevc")) {
                MediaFormat.MIMETYPE_VIDEO_HEVC
            } else {
                MediaFormat.MIMETYPE_VIDEO_AVC
            }
            
            // Create basic format - width/height will be updated by muxer
            val format = MediaFormat.createVideoFormat(mime, 1920, 1080)
            format.setInteger(MediaFormat.KEY_BIT_RATE, 5000000)
            format.setInteger(MediaFormat.KEY_FRAME_RATE, 30)
            format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
            
            // Extract codec-specific data (VPS/SPS/PPS for H.265, SPS/PPS for H.264)
            // These are typically in the first keyframe
            val csdData = extractCSD(data)
            if (csdData != null && csdData.isNotEmpty()) {
                val csdBuffer = ByteBuffer.wrap(csdData)
                format.setByteBuffer("csd-0", csdBuffer)
                Log.i(TAG, "Created MediaFormat with CSD: $mime, CSD size=${csdData.size}")
            } else {
                Log.i(TAG, "Created MediaFormat without CSD: $mime")
            }
            
            return format
        } catch (e: Exception) {
            Log.e(TAG, "Failed to extract media format", e)
            return null
        }
    }
    
    /**
     * Extract codec-specific data (VPS/SPS/PPS) from H.265 frame
     * Returns the parameter sets without start codes
     */
    private fun extractCSD(data: ByteArray): ByteArray? {
        try {
            val csdList = mutableListOf<ByteArray>()
            var i = 0
            
            while (i < data.size - 4) {
                // Find start code
                if (data[i] == 0.toByte() && data[i + 1] == 0.toByte()) {
                    val startCodeSize = if (data[i + 2] == 0.toByte() && data[i + 3] == 1.toByte()) 4 else if (data[i + 2] == 1.toByte()) 3 else 0
                    
                    if (startCodeSize > 0) {
                        val nalStart = i + startCodeSize
                        if (nalStart < data.size) {
                            val nalType = (data[nalStart].toInt() shr 1) and 0x3F // H.265 NAL unit type
                            
                            // H.265: VPS=32, SPS=33, PPS=34
                            if (nalType == 32 || nalType == 33 || nalType == 34) {
                                // Find next start code
                                var nalEnd = nalStart + 1
                                while (nalEnd < data.size - 3) {
                                    if (data[nalEnd] == 0.toByte() && data[nalEnd + 1] == 0.toByte() &&
                                        (data[nalEnd + 2] == 1.toByte() || (data[nalEnd + 2] == 0.toByte() && nalEnd + 3 < data.size && data[nalEnd + 3] == 1.toByte()))) {
                                        break
                                    }
                                    nalEnd++
                                }
                                
                                // Extract NAL unit without start code
                                val nalUnit = data.copyOfRange(nalStart, nalEnd)
                                csdList.add(nalUnit)
                            }
                        }
                        i += startCodeSize
                    } else {
                        i++
                    }
                } else {
                    i++
                }
            }
            
            // Combine all CSD NAL units with start codes
            if (csdList.isNotEmpty()) {
                val totalSize = csdList.sumOf { it.size + 4 } // +4 for start codes
                val result = ByteArray(totalSize)
                var offset = 0
                for (nal in csdList) {
                    // Add 4-byte start code
                    result[offset++] = 0
                    result[offset++] = 0
                    result[offset++] = 0
                    result[offset++] = 1
                    // Add NAL unit
                    System.arraycopy(nal, 0, result, offset, nal.size)
                    offset += nal.size
                }
                return result
            }
            
            return null
        } catch (e: Exception) {
            Log.e(TAG, "Failed to extract CSD", e)
            return null
        }
    }
    
    /**
     * Check if frame is a keyframe (IDR for H.264/265)
     */
    private fun isKeyFrame(data: ByteArray): Boolean {
        if (data.size < 5) return false
        
        // Check for Annex-B start code (0x00 0x00 0x00 0x01 or 0x00 0x00 0x01)
        val offset = if (data[0] == 0.toByte() && data[1] == 0.toByte() && 
                         data[2] == 0.toByte() && data[3] == 1.toByte()) {
            4  // 4-byte start code
        } else if (data[0] == 0.toByte() && data[1] == 0.toByte() && data[2] == 1.toByte()) {
            3  // 3-byte start code
        } else {
            return false  // No start code found
        }
        
        if (data.size < offset + 1) return false
        
        // For H.265/HEVC: NAL type is in upper 6 bits
        val nalType = (data[offset].toInt() shr 1) and 0x3F
        
        // Debug large frames
        if (data.size > 50000) {
            Log.d(TAG, "Large frame: ${data.size} bytes, NAL type: $nalType")
        }
        
        // H.265 NAL types:
        // 32 = VPS, 33 = SPS, 34 = PPS (parameter sets)
        // 19 = IDR_W_RADL, 20 = IDR_N_LP (keyframes)
        // 16-21 = various slice types
        // Large frames with VPS likely contain complete keyframe data (VPS+SPS+PPS+IDR)
        return nalType in 16..21 || (nalType in 32..34 && data.size > 50000)
    }
    
    /**
     * Stop recording and finalize MP4 file
     * @return File info (path and size) if successful
     */
    suspend fun stopRecording(): Pair<String, Long>? = withContext(Dispatchers.IO) {
        mutex.withLock {
            Log.d(TAG, "stopRecording called: isRecording=$isRecording, muxerStarted=$muxerStarted")
            
            if (!isRecording) {
                return@withLock null
            }
            
            try {
                // Stop and release muxer if it was started
                if (muxerStarted && muxer != null) {
                    Log.d(TAG, "Stopping muxer...")
                    muxer?.stop()
                    Log.d(TAG, "Muxer stopped")
                }
                if (muxer != null) {
                    Log.d(TAG, "Releasing muxer...")
                    muxer?.release()
                    Log.d(TAG, "Muxer released")
                }
                
                val file = currentFile
                val fileSize = file?.length() ?: 0L
                
                // Reset state
                muxer = null
                currentFile = null
                isRecording = false
                muxerStarted = false
                bytesWritten = 0L
                trackIndex = -1
                
                Log.i(TAG, "Stopped recording: ${file?.absolutePath} (${fileSize / 1024 / 1024} MB)")
                
                if (file != null && file.exists() && fileSize > 0) {
                    return@withLock Pair(file.absolutePath, fileSize)
                } else {
                    return@withLock null
                }
            } catch (e: Exception) {
                Log.e(TAG, "Failed to stop recording", e)
                // Reset state even on error
                muxer = null
                currentFile = null
                isRecording = false
                muxerStarted = false
                bytesWritten = 0L
                trackIndex = -1
                return@withLock null
            }
        }
    }
    
    /**
     * Check if currently recording
     */
    fun isRecording(): Boolean = isRecording
    
    /**
     * Get current recording file path
     */
    fun getCurrentFile(): String? = currentFile?.absolutePath
    
    /**
     * Get bytes written so far
     */
    fun getBytesWritten(): Long = bytesWritten
    
    private fun cleanup() {
        try {
            if (muxerStarted) {
                muxer?.stop()
            }
            muxer?.release()
        } catch (e: Exception) {
            Log.w(TAG, "Error releasing muxer", e)
        }
        muxer = null
        currentFile = null
        isRecording = false
        muxerStarted = false
        bytesWritten = 0L
        trackIndex = -1
    }
}
