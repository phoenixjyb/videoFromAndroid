package com.example.camviewer.network

import android.util.Log
import com.example.camviewer.data.model.ConnectionState
import com.example.camviewer.data.model.Telemetry
import com.example.camviewer.data.model.VideoFrame
import com.example.camviewer.di.KtorWebSocketClient
import io.ktor.client.*
import io.ktor.client.plugins.websocket.*
import io.ktor.websocket.*
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.serialization.json.Json
import javax.inject.Inject
import javax.inject.Singleton

private const val TAG = "CamControlWS"

/**
 * WebSocket client for receiving video frames and telemetry from CamControl.
 * 
 * Protocol:
 * - Binary frames: H.264/H.265 NAL units (Annex-B format with start codes)
 * - Text frames: JSON telemetry data
 * 
 * Connection URL: ws://<camcontrol-ip>:9090/control
 */
@Singleton
class CamControlWebSocketClient @Inject constructor(
    @KtorWebSocketClient private val client: HttpClient,
    private val json: Json
) {
    private val _connectionState = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()
    
    private val _videoFrames = MutableStateFlow<VideoFrame?>(null)
    val videoFrames: Flow<VideoFrame?> = _videoFrames
    
    private val _telemetry = MutableStateFlow<Telemetry?>(null)
    val telemetry: StateFlow<Telemetry?> = _telemetry.asStateFlow()
    
    private var currentSession: DefaultClientWebSocketSession? = null
    
    /**
     * Connect to CamControl WebSocket server
     * @param url WebSocket URL (e.g., "ws://192.168.1.100:9090")
     */
    suspend fun connect(url: String) {
        if (_connectionState.value is ConnectionState.Connected || 
            _connectionState.value is ConnectionState.Connecting) {
            return
        }
        
        Log.i(TAG, "Connecting to: $url/control")
        _connectionState.value = ConnectionState.Connecting
        
        try {
            client.webSocket(urlString = "$url/control") {
                currentSession = this
                _connectionState.value = ConnectionState.Connected
                Log.i(TAG, "WebSocket connected successfully")
                
                // Process incoming frames
                for (frame in incoming) {
                    when (frame) {
                        is Frame.Binary -> {
                            val data = frame.readBytes()
                            Log.d(TAG, "Received binary frame: ${data.size} bytes")
                            handleBinaryFrame(data)
                        }
                        is Frame.Text -> {
                            val text = frame.readText()
                            Log.d(TAG, "Received text frame: $text")
                            handleTextFrame(text)
                        }
                        is Frame.Close -> {
                            Log.i(TAG, "WebSocket closed")
                            _connectionState.value = ConnectionState.Disconnected
                            break
                        }
                        else -> { /* Ignore Ping, Pong */ }
                    }
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "WebSocket error: ${e.message}", e)
            _connectionState.value = ConnectionState.Error(e.message ?: "Connection failed")
        } finally {
            currentSession = null
            if (_connectionState.value !is ConnectionState.Error) {
                _connectionState.value = ConnectionState.Disconnected
            }
        }
    }
    
    /**
     * Disconnect from WebSocket server
     */
    suspend fun disconnect() {
        currentSession?.close(CloseReason(CloseReason.Codes.NORMAL, "Client disconnect"))
        currentSession = null
        _connectionState.value = ConnectionState.Disconnected
    }
    
    /**
     * Send camera control command
     * @param command Command name (e.g., "zoom", "focus", "codec")
     * @param params Command parameters
     */
    suspend fun sendCommand(command: String, params: Map<String, String> = emptyMap()) {
        currentSession?.let { session ->
            if (session.isActive) {
                try {
                    val commandJson = json.encodeToString(
                        kotlinx.serialization.serializer(),
                        mapOf("command" to command, "params" to params)
                    )
                    session.send(Frame.Text(commandJson))
                } catch (e: Exception) {
                    // Log error
                }
            }
        }
    }
    
    /**
     * Handle binary frame (video data)
     * Binary frames contain H.264/H.265 NAL units in Annex-B format
     */
    private fun handleBinaryFrame(data: ByteArray) {
        // Check if this is a keyframe by looking for SPS/PPS NAL units
        val isKeyframe = isKeyframeData(data)
        
        if (isKeyframe) {
            Log.i(TAG, "Received KEYFRAME: ${data.size} bytes")
        }
        
        val videoFrame = VideoFrame(
            data = data,
            timestamp = System.currentTimeMillis(),
            isKeyframe = isKeyframe
        )
        
        _videoFrames.value = videoFrame
    }
    
    /**
     * Handle text frame (telemetry data)
     */
    private fun handleTextFrame(text: String) {
        try {
            val telemetry = json.decodeFromString<Telemetry>(text)
            _telemetry.value = telemetry
        } catch (e: Exception) {
            // Log error - invalid JSON
        }
    }
    
    /**
     * Check if data contains keyframe indicators (SPS/PPS NAL units)
     * H.264 SPS = NAL type 7, PPS = NAL type 8
     * H.265 SPS = NAL type 33, PPS = NAL type 34
     */
    private fun isKeyframeData(data: ByteArray): Boolean {
        var i = 0
        while (i < data.size - 4) {
            // Look for start code 0x00 0x00 0x00 0x01
            if (data[i] == 0.toByte() && 
                data[i + 1] == 0.toByte() && 
                data[i + 2] == 0.toByte() && 
                data[i + 3] == 1.toByte()) {
                
                if (i + 4 < data.size) {
                    val nalType = (data[i + 4].toInt() and 0x1F) // H.264 NAL type (lower 5 bits)
                    val nalTypeH265 = (data[i + 4].toInt() and 0x7E) shr 1 // H.265 NAL type (bits 1-6)
                    
                    // H.264: SPS=7, PPS=8, IDR=5
                    // H.265: SPS=33, PPS=34, IDR=19-20
                    if (nalType == 7 || nalType == 8 || nalType == 5 ||
                        nalTypeH265 == 33 || nalTypeH265 == 34 || 
                        nalTypeH265 == 19 || nalTypeH265 == 20) {
                        return true
                    }
                }
                i += 4
            } else {
                i++
            }
        }
        return false
    }
}
