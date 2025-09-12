package com.example.camcontrol.encode

import android.content.Context
import android.media.MediaCodec
import android.media.MediaFormat
import android.media.MediaMuxer
import android.os.Environment
import android.util.Log
import java.io.File
import java.nio.ByteBuffer

class VideoRecorder(private val context: Context) : VideoEncoder.MuxerSink {
    private var muxer: MediaMuxer? = null
    private var trackIndex: Int = -1
    private var isStarted = false
    private var outputFile: File? = null

    companion object { private const val TAG = "VideoRecorder" }

    fun startRecording(nameHint: String? = null): File {
        if (muxer != null) stopRecording()
        val dir = context.getExternalFilesDir(Environment.DIRECTORY_MOVIES) ?: context.filesDir
        val safeName = (nameHint?.ifBlank { null } ?: "clip")
            .replace(Regex("[^a-zA-Z0-9_-]"), "_")
        val file = File(dir, "$safeName-${System.currentTimeMillis()}.mp4")
        muxer = MediaMuxer(file.absolutePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
        trackIndex = -1
        isStarted = false
        outputFile = file
        Log.i(TAG, "Recording to ${file.absolutePath}")
        return file
    }

    fun stopRecording(): File? {
        try {
            if (isStarted) muxer?.stop()
        } catch (t: Throwable) {
            Log.w(TAG, "muxer stop error", t)
        }
        try {
            muxer?.release()
        } catch (t: Throwable) {
            Log.w(TAG, "muxer release error", t)
        }
        val f = outputFile
        muxer = null
        outputFile = null
        trackIndex = -1
        isStarted = false
        return f
    }

    override fun onFormatChanged(format: MediaFormat) {
        val m = muxer ?: return
        if (trackIndex == -1) {
            trackIndex = m.addTrack(format)
        }
        if (!isStarted && trackIndex >= 0) {
            m.start()
            isStarted = true
        }
    }

    override fun onSample(sample: ByteBuffer, bufferInfo: MediaCodec.BufferInfo) {
        val m = muxer ?: return
        if (!isStarted || trackIndex < 0) return
        try {
            m.writeSampleData(trackIndex, sample, bufferInfo)
        } catch (t: Throwable) {
            Log.w(TAG, "writeSampleData error", t)
        }
    }
}

