package com.example.camcontrol.encode

// Mirrored into module path for build

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.util.Log
import android.view.Surface
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.nio.ByteBuffer

class VideoEncoder(
    private val scope: CoroutineScope,
    private val onEncodedAnnexB: (ByteArray) -> Unit
) {
    private var mediaCodec: MediaCodec? = null
    private var _inputSurface: Surface? = null
    val inputSurface: Surface?
        get() = _inputSurface

    @Volatile private var codecConfigAnnexB: ByteArray? = null
    @Volatile private var muxerSink: MuxerSink? = null
    @Volatile private var lastOutputFormat: MediaFormat? = null
    @Volatile private var drainJob: Job? = null

    companion object {
        private const val TAG = "VideoEncoder"
        private const val MIME_TYPE = MediaFormat.MIMETYPE_VIDEO_AVC
        private const val I_FRAME_INTERVAL = 2
        private val START_CODE = byteArrayOf(0x00, 0x00, 0x00, 0x01)
    }

    private var width = 1920
    private var height = 1080
    private var frameRate = 30
    private var bitRate = 8 * 1024 * 1024

    fun configure(width: Int, height: Int, fps: Int, bitrate: Int) {
        this.width = width
        this.height = height
        this.frameRate = fps
        this.bitRate = bitrate
    }

    fun start() {
        Log.d(TAG, "Starting encoder...")
        val format = MediaFormat.createVideoFormat(MIME_TYPE, width, height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, bitRate)
            setInteger(MediaFormat.KEY_FRAME_RATE, frameRate)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, I_FRAME_INTERVAL)
            // Make stream Broadway-friendly: Baseline profile, CBR if available
            try { setInteger(MediaFormat.KEY_PROFILE, MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline) } catch (_: Throwable) {}
            try { setInteger(MediaFormat.KEY_LEVEL, MediaCodecInfo.CodecProfileLevel.AVCLevel31) } catch (_: Throwable) {}
            try { setInteger(MediaFormat.KEY_BITRATE_MODE, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR) } catch (_: Throwable) {}
            // Prepend SPS/PPS to IDR (framework key where supported)
            try { setInteger(MediaFormat.KEY_PREPEND_HEADER_TO_SYNC_FRAMES, 1) } catch (_: Throwable) {}
            // Qualcomm-specific hints (ignored on others)
            try { setInteger("vendor.qti-ext-enc-profile-level.profile", MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline) } catch (_: Throwable) {}
            try { setInteger("vendor.qti-ext-enc-profile-level.level", MediaCodecInfo.CodecProfileLevel.AVCLevel31) } catch (_: Throwable) {}
        }

        // Cancel any existing drain job to prevent concurrent access
        drainJob?.cancel()
        
        mediaCodec = MediaCodec.createEncoderByType(MIME_TYPE).apply {
            configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            _inputSurface = createInputSurface()
            start()
        }

        drainJob = scope.launch(Dispatchers.IO) { drainEncoder() }
        Log.d(TAG, "Encoder started.")
    }

    private fun drainEncoder() {
        val bufferInfo = MediaCodec.BufferInfo()
        while (scope.isActive) {
            try {
                val codec = mediaCodec ?: break
                // Use shorter timeout to avoid blocking, but add delay between calls
                val outIndex = codec.dequeueOutputBuffer(bufferInfo, 1000)
            if (outIndex >= 0) {
                val outputBuffer = codec.getOutputBuffer(outIndex)
                if (outputBuffer != null && bufferInfo.size > 0) {
                    outputBuffer.position(bufferInfo.offset)
                    outputBuffer.limit(bufferInfo.offset + bufferInfo.size)
                    val isCodecConfig = (bufferInfo.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0
                    if (!isCodecConfig) {
                        val sample = ByteArray(bufferInfo.size)
                        outputBuffer.get(sample)
                        val annexBFrame = convertToAnnexB(sample)
                        val isKeyFrame = (bufferInfo.flags and MediaCodec.BUFFER_FLAG_KEY_FRAME) != 0
                        val prefixed = if (isKeyFrame) {
                            val csd = codecConfigAnnexB
                            if (csd != null && csd.isNotEmpty()) csd + annexBFrame else annexBFrame
                        } else annexBFrame
                        onEncodedAnnexB(prefixed)

                        // Also feed raw sample to muxer sink for MP4
                        muxerSink?.let { sink ->
                            val dupInfo = MediaCodec.BufferInfo().apply {
                                set(bufferInfo.offset, bufferInfo.size, bufferInfo.presentationTimeUs, bufferInfo.flags)
                            }
                            sink.onSample(ByteBuffer.wrap(sample), dupInfo)
                        }
                    }
                }
                codec.releaseOutputBuffer(outIndex, false)
            } else if (outIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                val format = codec.outputFormat
                val csd0 = format.getByteBuffer("csd-0")
                val csd1 = format.getByteBuffer("csd-1")
                val parts = ArrayList<ByteArray>()
                if (csd0 != null) parts.add(convertToAnnexB(byteBufferToArray(csd0)))
                if (csd1 != null) parts.add(convertToAnnexB(byteBufferToArray(csd1)))
                codecConfigAnnexB = parts.fold(ByteArray(0)) { acc, bytes -> acc + bytes }
                Log.d(TAG, "Output format: $format")

                lastOutputFormat = format
                muxerSink?.onFormatChanged(format)
            } else if (outIndex == MediaCodec.INFO_TRY_AGAIN_LATER) {
                // No output available yet, wait before trying again
                Thread.sleep(10)
            } else {
                // Negative value we don't handle, wait a bit
                Thread.sleep(5)
            }
            } catch (e: Exception) {
                Log.w(TAG, "Error in drainEncoder: ${e.message}")
                Thread.sleep(200) // Longer wait on errors to prevent spam
            }
        }
    }

    fun stop() {
        Log.d(TAG, "Stopping encoder.")
        drainJob?.cancel()
        drainJob = null
        try { mediaCodec?.signalEndOfInputStream() } catch (_: Throwable) {}
        mediaCodec?.stop()
        mediaCodec?.release()
        mediaCodec = null
        _inputSurface = null
        codecConfigAnnexB = null
    }

    private fun byteBufferToArray(bb: ByteBuffer): ByteArray {
        val dup = bb.duplicate(); dup.clear()
        val arr = ByteArray(dup.remaining())
        dup.get(arr)
        return arr
    }

    private fun convertToAnnexB(sample: ByteArray): ByteArray {
        if (sample.size >= 4 && sample[0] == 0.toByte() && sample[1] == 0.toByte() && sample[2] == 0.toByte() && sample[3] == 1.toByte()) {
            return sample
        }
        var offset = 0
        val out = ArrayList<ByteArray>()
        while (offset + 4 <= sample.size) {
            val length = ((sample[offset].toInt() and 0xFF) shl 24) or
                    ((sample[offset + 1].toInt() and 0xFF) shl 16) or
                    ((sample[offset + 2].toInt() and 0xFF) shl 8) or
                    (sample[offset + 3].toInt() and 0xFF)
            offset += 4
            if (length <= 0 || offset + length > sample.size) break
            out.add(START_CODE + sample.copyOfRange(offset, offset + length))
            offset += length
        }
        return out.fold(ByteArray(0)) { acc, bytes -> acc + bytes }
    }

    fun setMuxerSink(sink: MuxerSink?) {
        muxerSink = sink
        // If a sink attaches after encoder has started, provide format immediately
        val fmt = lastOutputFormat
        if (sink != null && fmt != null) {
            try { sink.onFormatChanged(fmt) } catch (_: Throwable) {}
        }
    }

    interface MuxerSink {
        fun onFormatChanged(format: MediaFormat)
        fun onSample(sample: ByteBuffer, bufferInfo: MediaCodec.BufferInfo)
    }
}
