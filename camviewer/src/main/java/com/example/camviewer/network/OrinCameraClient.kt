package com.example.camviewer.network

import android.util.Log
import io.ktor.client.*
import io.ktor.client.request.*
import io.ktor.client.statement.*
import io.ktor.http.*
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json

/**
 * Client for sending camera control commands to Orin WebSocket server
 * Mirrors the command format from camControl app
 */
class OrinCameraClient(
    private val httpClient: HttpClient,
    private val json: Json
) {
    private val TAG = "OrinCameraClient"
    
    /**
     * Send camera control command via WebSocket control endpoint
     * Commands are sent as JSON to ws://<orin>:8080/control
     */
    private suspend fun sendCommand(orinHost: String, command: CameraCommand): Result<Unit> {
        return try {
            val jsonCommand = json.encodeToString(command)
            Log.d(TAG, "Sending command: $jsonCommand")
            
            // Send as text message to WebSocket endpoint via HTTP POST
            // The actual WebSocket handling is done by the Orin server
            val response: HttpResponse = httpClient.post("http://$orinHost:8080/control") {
                contentType(ContentType.Application.Json)
                setBody(jsonCommand)
            }
            
            if (response.status.isSuccess()) {
                Log.d(TAG, "Command sent successfully")
                Result.success(Unit)
            } else {
                Log.e(TAG, "Command failed: ${response.status}")
                Result.failure(Exception("HTTP ${response.status}"))
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to send command", e)
            Result.failure(e)
        }
    }
    
    suspend fun setZoom(orinHost: String, value: Float) =
        sendCommand(orinHost, CameraCommand.SetZoomRatio(value))
    
    suspend fun setAeLock(orinHost: String, enabled: Boolean) =
        sendCommand(orinHost, CameraCommand.SetAeLock(enabled))
    
    suspend fun setAwbLock(orinHost: String, enabled: Boolean) =
        sendCommand(orinHost, CameraCommand.SetAwbLock(enabled))
    
    suspend fun switchCamera(orinHost: String, facing: String) =
        sendCommand(orinHost, CameraCommand.SwitchCamera(facing))
    
    suspend fun setBitrate(orinHost: String, bitrate: Int) =
        sendCommand(orinHost, CameraCommand.SetBitrate(bitrate))
    
    suspend fun setCodec(orinHost: String, codec: String) =
        sendCommand(orinHost, CameraCommand.SetCodec(codec))
    
    suspend fun setVideoProfile(orinHost: String, width: Int, height: Int, fps: Int) =
        sendCommand(orinHost, CameraCommand.SetVideoProfile(width, height, fps))
    
    suspend fun requestKeyFrame(orinHost: String) =
        sendCommand(orinHost, CameraCommand.RequestKeyFrame)
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
