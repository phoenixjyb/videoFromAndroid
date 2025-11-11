package com.example.camviewer.ui.screens.video

import android.media.MediaFormat
import android.util.Log
import android.view.Surface
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.camviewer.data.model.ConnectionState
import com.example.camviewer.data.model.Telemetry
import com.example.camviewer.data.repository.SettingsRepository
import com.example.camviewer.network.CamControlWebSocketClient
import com.example.camviewer.video.VideoDecoder
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

private const val TAG = "VideoViewModel"

/**
 * ViewModel for video streaming screen.
 * Manages WebSocket connection, video decoding, and telemetry display.
 */
@HiltViewModel
class VideoViewModel @Inject constructor(
    private val webSocketClient: CamControlWebSocketClient,
    private val videoDecoder: VideoDecoder,
    private val settingsRepository: SettingsRepository
) : ViewModel() {
    
    val connectionState: StateFlow<ConnectionState> = webSocketClient.connectionState
    val telemetry: StateFlow<Telemetry?> = webSocketClient.telemetry
    
    private val _latency = MutableStateFlow(0L)
    val latency: StateFlow<Long> = _latency.asStateFlow()
    
    private var decodingJob: Job? = null
    private var surface: Surface? = null
    private var isDecoderInitialized = false
    private var hasReceivedKeyframe = false
    
    /**
     * Initialize video decoder with surface
     */
    fun initializeDecoder(surface: Surface, width: Int = 1920, height: Int = 1080) {
        Log.i(TAG, "Initializing decoder: ${width}x${height}")
        this.surface = surface
        
        viewModelScope.launch {
            try {
                videoDecoder.initialize(
                    codecType = MediaFormat.MIMETYPE_VIDEO_HEVC, // H.265
                    width = width,
                    height = height,
                    surface = surface
                )
                isDecoderInitialized = true
                Log.i(TAG, "Decoder initialized successfully")
                startDecoding()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to initialize decoder", e)
            }
        }
    }
    
    /**
     * Connect to CamControl server
     */
    fun connect() {
        viewModelScope.launch {
            settingsRepository.settings.first().let { settings ->
                webSocketClient.connect(settings.cameraUrl)
            }
        }
    }
    
    /**
     * Disconnect from server
     */
    fun disconnect() {
        viewModelScope.launch {
            stopDecoding()
            webSocketClient.disconnect()
        }
    }
    
    /**
     * Start decoding video frames
     */
    private fun startDecoding() {
        if (!isDecoderInitialized) {
            Log.w(TAG, "Cannot start decoding - decoder not initialized")
            return
        }
        
        Log.i(TAG, "Starting video decoding pipeline")
        hasReceivedKeyframe = false
        decodingJob?.cancel()
        decodingJob = viewModelScope.launch {
            var frameCount = 0
            webSocketClient.videoFrames
                .filterNotNull()
                .collect { frame ->
                    // Wait for first keyframe before decoding
                    if (!hasReceivedKeyframe) {
                        if (frame.isKeyframe) {
                            Log.i(TAG, "Received first keyframe, starting decoding")
                            hasReceivedKeyframe = true
                        } else {
                            Log.d(TAG, "Skipping frame until keyframe arrives")
                            return@collect
                        }
                    }
                    
                    frameCount++
                    if (frameCount % 30 == 0) { // Log every 30 frames
                        Log.d(TAG, "Decoded $frameCount frames, latest size: ${frame.data.size} bytes, keyframe: ${frame.isKeyframe}")
                    }
                    
                    // Calculate latency
                    val now = System.currentTimeMillis()
                    _latency.value = now - frame.timestamp
                    
                    // Decode frame
                    val timestampUs = frame.timestamp * 1000 // Convert ms to us
                    val success = videoDecoder.decodeFrame(frame.data, timestampUs)
                    
                    if (!success && frameCount <= 10) {
                        Log.w(TAG, "Failed to decode frame $frameCount")
                    }
                }
        }
    }
    
    /**
     * Stop decoding
     */
    private fun stopDecoding() {
        decodingJob?.cancel()
        decodingJob = null
        hasReceivedKeyframe = false
    }
    
    /**
     * Send camera control command
     */
    fun sendCommand(command: String, params: Map<String, String> = emptyMap()) {
        viewModelScope.launch {
            webSocketClient.sendCommand(command, params)
        }
    }
    
    /**
     * Send target coordinates (x, y normalized 0.0-1.0)
     */
    fun sendTargetCoordinates(x: Float, y: Float) {
        // Will be implemented in Phase 3
    }
    
    /**
     * Release resources
     */
    override fun onCleared() {
        super.onCleared()
        viewModelScope.launch {
            stopDecoding()
            videoDecoder.release()
            webSocketClient.disconnect()
        }
    }
}
