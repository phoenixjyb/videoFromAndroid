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
import com.example.camviewer.network.OrinTargetClient
import com.example.camviewer.network.PhoneCameraClient
import com.example.camviewer.video.VideoDecoder
import com.example.camviewer.video.VideoRecorder
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
    private val videoRecorder: VideoRecorder,
    private val settingsRepository: SettingsRepository,
    private val orinTargetClient: OrinTargetClient,
    private val phoneCameraClient: PhoneCameraClient
) : ViewModel() {
    
    val connectionState: StateFlow<ConnectionState> = webSocketClient.connectionState
    val telemetry: StateFlow<Telemetry?> = webSocketClient.telemetry
    
    private val _latency = MutableStateFlow(0L)
    val latency: StateFlow<Long> = _latency.asStateFlow()
    
    private val _isRecording = MutableStateFlow(false)
    val isRecording: StateFlow<Boolean> = _isRecording.asStateFlow()
    
    private val _recordingFile = MutableStateFlow<String?>(null)
    val recordingFile: StateFlow<String?> = _recordingFile.asStateFlow()
    
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
                    
                    // Write frame to recorder if recording
                    if (_isRecording.value) {
                        videoRecorder.writeFrame(frame.data)
                    }
                    
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
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                val result = orinTargetClient.sendTargetCoordinates(
                    baseUrl = settings.orinTargetUrl,
                    x = x,
                    y = y
                )
                if (result.isSuccess) {
                    Log.i(TAG, "Target coordinates sent: ($x, $y)")
                } else {
                    Log.e(TAG, "Failed to send target coordinates: ${result.exceptionOrNull()?.message}")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error sending target coordinates", e)
            }
        }
    }
    
    /**
     * Send target ROI (Region of Interest) with bounding box
     * 
     * @param x Top-left x coordinate (normalized 0.0-1.0)
     * @param y Top-left y coordinate (normalized 0.0-1.0)
     * @param width ROI width (normalized 0.0-1.0)
     * @param height ROI height (normalized 0.0-1.0)
     */
    fun sendTargetROI(x: Float, y: Float, width: Float, height: Float) {
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                val result = orinTargetClient.sendTargetROI(
                    baseUrl = settings.orinTargetUrl,
                    x = x,
                    y = y,
                    width = width,
                    height = height
                )
                if (result.isSuccess) {
                    Log.i(TAG, "Target ROI sent: x=$x, y=$y, w=$width, h=$height")
                } else {
                    Log.e(TAG, "Failed to send target ROI: ${result.exceptionOrNull()?.message}")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error sending target ROI", e)
            }
        }
    }
    
    /**
     * Developer mode state from settings
     */
    val developerModeEnabled: Flow<Boolean> = settingsRepository.settings
        .map { it.developerModeEnabled }
    
    /**
     * Camera control commands (developer mode features)
     * Send commands to phone's camera via WebSocket
     */
    fun setZoom(value: Float) {
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                Log.d(TAG, "Setting zoom to $value on phone ${settings.phoneControlHost}")
                phoneCameraClient.setZoom(settings.phoneControlHost, value)
            } catch (e: Exception) {
                Log.e(TAG, "Error setting zoom", e)
            }
        }
    }
    
    fun setAeLock(enabled: Boolean) {
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                Log.d(TAG, "Setting AE lock to $enabled on phone ${settings.phoneControlHost}")
                phoneCameraClient.setAeLock(settings.phoneControlHost, enabled)
            } catch (e: Exception) {
                Log.e(TAG, "Error setting AE lock", e)
            }
        }
    }
    
    fun setAwbLock(enabled: Boolean) {
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                Log.d(TAG, "Setting AWB lock to $enabled on phone ${settings.phoneControlHost}")
                phoneCameraClient.setAwbLock(settings.phoneControlHost, enabled)
            } catch (e: Exception) {
                Log.e(TAG, "Error setting AWB lock", e)
            }
        }
    }
    
    fun switchCamera(facing: String) {
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                Log.d(TAG, "Switching camera to $facing on phone ${settings.phoneControlHost}")
                phoneCameraClient.switchCamera(settings.phoneControlHost, facing)
            } catch (e: Exception) {
                Log.e(TAG, "Error switching camera", e)
            }
        }
    }
    
    fun setBitrate(bitrate: Int) {
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                Log.d(TAG, "Setting bitrate to ${bitrate / 1000000}Mbps on phone ${settings.phoneControlHost}")
                phoneCameraClient.setBitrate(settings.phoneControlHost, bitrate)
            } catch (e: Exception) {
                Log.e(TAG, "Error setting bitrate", e)
            }
        }
    }
    
    fun setCodec(codec: String) {
        viewModelScope.launch {
            try {
                val settings = settingsRepository.settings.first()
                Log.d(TAG, "Setting codec to $codec on phone ${settings.phoneControlHost}")
                phoneCameraClient.setCodec(settings.phoneControlHost, codec)
            } catch (e: Exception) {
                Log.e(TAG, "Error setting codec", e)
            }
        }
    }
    
    /**
     * Start recording video to file
     */
    fun startRecording() {
        if (_isRecording.value) {
            Log.w(TAG, "Already recording")
            return
        }
        
        viewModelScope.launch {
            try {
                // Determine codec from current stream (default to h265)
                val codec = "h265" // TODO: Track actual codec from stream
                val filePath = videoRecorder.startRecording(codec)
                if (filePath != null) {
                    _isRecording.value = true
                    _recordingFile.value = filePath
                    Log.i(TAG, "Started recording to: $filePath")
                } else {
                    Log.e(TAG, "Failed to start recording")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error starting recording", e)
            }
        }
    }
    
    /**
     * Stop recording video
     */
    fun stopRecording() {
        if (!_isRecording.value) {
            Log.w(TAG, "Not recording")
            return
        }
        
        viewModelScope.launch {
            try {
                val result = videoRecorder.stopRecording()
                _isRecording.value = false
                
                if (result != null) {
                    val (filePath, bytes) = result
                    Log.i(TAG, "Stopped recording: $filePath (${bytes / 1024 / 1024} MB)")
                    _recordingFile.value = filePath
                } else {
                    _recordingFile.value = null
                    Log.e(TAG, "Failed to stop recording")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error stopping recording", e)
                _isRecording.value = false
                _recordingFile.value = null
            }
        }
    }
    
    /**
     * Toggle recording on/off
     */
    fun toggleRecording() {
        if (_isRecording.value) {
            stopRecording()
        } else {
            startRecording()
        }
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
