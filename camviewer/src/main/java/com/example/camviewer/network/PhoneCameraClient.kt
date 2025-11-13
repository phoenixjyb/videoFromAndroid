package com.example.camviewer.network

import android.util.Log
import io.ktor.client.*
import io.ktor.client.plugins.websocket.*
import io.ktor.websocket.*
import kotlinx.coroutines.channels.Channel
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json

/**
 * Client for sending camera control commands to phone's WebSocket server
 * Connects to the camControl app running on the phone (SM-S9280)
 * 
 * IMPORTANT: Command format must match phone's expectation:
 * - Uses "cmd" as discriminator field (not "type")
 * - Command names: "zoom", "ae_lock", "awb_lock", "switch", "bitrate", "codec", etc.
 */
class PhoneCameraClient(
    private val httpClient: HttpClient,
    private val json: Json
) {
    private val TAG = "PhoneCameraClient"
    
    // Json with "cmd" discriminator to match phone app
    private val commandJson = Json {
        classDiscriminator = "cmd"
        ignoreUnknownKeys = true
    }
    
    /**
     * Send camera control command via WebSocket
     * Commands are sent as JSON to ws://<phone-ip>:9090/control
     */
    suspend fun sendCommand(phoneHost: String, command: CameraCommand): Result<Unit> {
        return try {
            val jsonCommand = commandJson.encodeToString(command)
            Log.d(TAG, "Sending command to $phoneHost:9090/control: $jsonCommand")
            
            // Connect to phone's WebSocket server and send command
            httpClient.webSocket(
                host = phoneHost,
                port = 9090,
                path = "/control"
            ) {
                // Send the command as text frame
                send(Frame.Text(jsonCommand))
                Log.d(TAG, "Command sent successfully")
            }
            
            Result.success(Unit)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to send command to $phoneHost", e)
            Result.failure(e)
        }
    }
    
    suspend fun setZoom(phoneHost: String, value: Float) =
        sendCommand(phoneHost, CameraCommand.SetZoomRatio(value))
    
    suspend fun setAeLock(phoneHost: String, enabled: Boolean) =
        sendCommand(phoneHost, CameraCommand.SetAeLock(enabled))
    
    suspend fun setAwbLock(phoneHost: String, enabled: Boolean) =
        sendCommand(phoneHost, CameraCommand.SetAwbLock(enabled))
    
    suspend fun switchCamera(phoneHost: String, facing: String) =
        sendCommand(phoneHost, CameraCommand.SwitchCamera(facing))
    
    suspend fun setBitrate(phoneHost: String, bitrate: Int) =
        sendCommand(phoneHost, CameraCommand.SetBitrate(bitrate))
    
    suspend fun setCodec(phoneHost: String, codec: String) =
        sendCommand(phoneHost, CameraCommand.SetCodec(codec))
    
    suspend fun setVideoProfile(phoneHost: String, width: Int, height: Int, fps: Int) =
        sendCommand(phoneHost, CameraCommand.SetVideoProfile(width, height, fps))
    
    suspend fun requestKeyFrame(phoneHost: String) =
        sendCommand(phoneHost, CameraCommand.RequestKeyFrame)

    /**
     * Camera control commands - matches the phone app's format
     * Uses "cmd" as discriminator field with specific command names
     */
    @Serializable
    sealed class CameraCommand {
        @Serializable
        @SerialName("setZoomRatio")
        data class SetZoomRatio(val value: Float) : CameraCommand()
        
        @Serializable
        @SerialName("setAeLock")
        data class SetAeLock(val value: Boolean) : CameraCommand()
        
        @Serializable
        @SerialName("setAwbLock")
        data class SetAwbLock(val value: Boolean) : CameraCommand()
        
        @Serializable
        @SerialName("switchCamera")
        data class SwitchCamera(val facing: String) : CameraCommand()  // "back" or "front"
        
        @Serializable
        @SerialName("setBitrate")
        data class SetBitrate(val bitrate: Int) : CameraCommand()  // Bitrate in bits per second
        
        @Serializable
        @SerialName("setCodec")
        data class SetCodec(val codec: String) : CameraCommand()  // "h264" or "h265"
        
        @Serializable
        @SerialName("setVideoProfile")
        data class SetVideoProfile(val width: Int, val height: Int, val fps: Int, val highSpeed: Boolean = false) : CameraCommand()
        
        @Serializable
        @SerialName("requestKeyFrame")
        object RequestKeyFrame : CameraCommand()
    }
}
