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
 * Mirrors the command format from camControl app's ControlCommand.kt
 */
class PhoneCameraClient(
    private val httpClient: HttpClient,
    private val json: Json
) {
    private val TAG = "PhoneCameraClient"
    private val commandChannel = Channel<CameraCommand>(Channel.UNLIMITED)
    
    /**
     * Send camera control command via WebSocket
     * Commands are sent as JSON to ws://<phone-ip>:8080
     */
    suspend fun sendCommand(phoneHost: String, command: CameraCommand): Result<Unit> {
        return try {
            val jsonCommand = json.encodeToString(command)
            Log.d(TAG, "Sending command to $phoneHost:8080: $jsonCommand")
            
            // Connect to phone's WebSocket server and send command
            httpClient.webSocket(
                host = phoneHost,
                port = 8080,
                path = "/"
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
}

/**
 * Camera control commands - matches the format from camControl app
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
    data class SwitchCamera(val facing: String) : CameraCommand()
    
    @Serializable
    @SerialName("setBitrate")
    data class SetBitrate(val bitrate: Int) : CameraCommand()
    
    @Serializable
    @SerialName("setCodec")
    data class SetCodec(val codec: String) : CameraCommand()
    
    @Serializable
    @SerialName("setVideoProfile")
    data class SetVideoProfile(
        val width: Int,
        val height: Int,
        val fps: Int,
        val highSpeed: Boolean = false
    ) : CameraCommand()
    
    @Serializable
    @SerialName("requestKeyFrame")
    object RequestKeyFrame : CameraCommand()
}
