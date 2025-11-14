package com.example.camviewer.data.model

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/**
 * Connection state for video streaming
 */
sealed class ConnectionState {
    object Disconnected : ConnectionState()
    object Connecting : ConnectionState()
    object Connected : ConnectionState()
    data class Error(val message: String) : ConnectionState()
}

/**
 * Video frame data from WebSocket
 */
data class VideoFrame(
    val data: ByteArray,
    val timestamp: Long = System.currentTimeMillis(),
    val isKeyframe: Boolean = false
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as VideoFrame
        if (!data.contentEquals(other.data)) return false
        if (timestamp != other.timestamp) return false
        if (isKeyframe != other.isKeyframe) return false
        return true
    }

    override fun hashCode(): Int {
        var result = data.contentHashCode()
        result = 31 * result + timestamp.hashCode()
        result = 31 * result + isKeyframe.hashCode()
        return result
    }
}

/**
 * Telemetry data sent with video frames
 */
@Serializable
data class Telemetry(
    @SerialName("frame_number") val frameNumber: Int = 0,
    @SerialName("timestamp_ms") val timestampMs: Long = 0,
    @SerialName("fps") val fps: Float = 0f,
    @SerialName("bitrate_kbps") val bitrateKbps: Int = 0,
    @SerialName("resolution") val resolution: Resolution? = null,
    @SerialName("codec") val codec: String = "",
    @SerialName("tracking_data") val trackingData: TrackingData? = null
)

@Serializable
data class Resolution(
    @SerialName("width") val width: Int,
    @SerialName("height") val height: Int
)

@Serializable
data class TrackingData(
    @SerialName("bbox") val bbox: BoundingBox? = null,
    @SerialName("confidence") val confidence: Float = 0f,
    @SerialName("track_id") val trackId: Int? = null
)

@Serializable
data class BoundingBox(
    @SerialName("x") val x: Float,
    @SerialName("y") val y: Float,
    @SerialName("width") val width: Float,
    @SerialName("height") val height: Float
)

/**
 * Camera control commands
 */
@Serializable
data class CameraCommand(
    @SerialName("command") val command: String,
    @SerialName("params") val params: Map<String, String> = emptyMap()
)

/**
 * Target selection coordinates
 */
@Serializable
data class TargetCoordinates(
    @SerialName("x") val x: Float,
    @SerialName("y") val y: Float,
    @SerialName("timestamp") val timestamp: Long = System.currentTimeMillis()
)

/**
 * Network preset options
 */
enum class NetworkPreset(
    val displayName: String,
    val phoneIp: String,
    val orinIp: String,
    val tabletIp: String
) {
    ZEROTIER(
        displayName = "ZeroTier",
        phoneIp = "192.168.100.156",
        orinIp = "192.168.100.150",
        tabletIp = "192.168.100.159"
    ),
    T8SPACE(
        displayName = "T8Space",
        phoneIp = "172.16.30.28",
        orinIp = "172.16.30.234",
        tabletIp = "172.16.30.199"
    ),
    CUSTOM(
        displayName = "Custom",
        phoneIp = "",
        orinIp = "",
        tabletIp = ""
    );
    
    companion object {
        const val PHONE_WS_PORT = 9090
        const val ORIN_TARGET_PORT = 8082
        const val ORIN_MEDIA_PORT = 8081
        
        fun fromName(name: String): NetworkPreset {
            return values().find { it.name == name } ?: ZEROTIER
        }
    }
    
    fun getPhoneVideoUrl(): String = "ws://$phoneIp:$PHONE_WS_PORT/"
    fun getPhoneControlUrl(): String = "ws://$phoneIp:$PHONE_WS_PORT/control"
    fun getOrinTargetUrl(): String = "http://$orinIp:$ORIN_TARGET_PORT"
    fun getOrinMediaUrl(): String = "http://$orinIp:$ORIN_MEDIA_PORT"
}

/**
 * Settings/preferences data
 */
data class AppSettings(
    val networkPreset: NetworkPreset = NetworkPreset.ZEROTIER,
    val cameraUrl: String = NetworkPreset.ZEROTIER.getPhoneVideoUrl(),
    val orinTargetUrl: String = NetworkPreset.ZEROTIER.getOrinTargetUrl(),
    val orinMediaUrl: String = NetworkPreset.ZEROTIER.getOrinMediaUrl(),
    val phoneControlHost: String = NetworkPreset.ZEROTIER.phoneIp,
    val developerModeEnabled: Boolean = false,
    val serviceControlPin: String = "" // PIN for Orin service control (empty = no PIN)
) {
    companion object {
        fun fromPreset(preset: NetworkPreset, developerModeEnabled: Boolean = false, serviceControlPin: String = ""): AppSettings {
            return AppSettings(
                networkPreset = preset,
                cameraUrl = preset.getPhoneVideoUrl(),
                orinTargetUrl = preset.getOrinTargetUrl(),
                orinMediaUrl = preset.getOrinMediaUrl(),
                phoneControlHost = preset.phoneIp,
                developerModeEnabled = developerModeEnabled,
                serviceControlPin = serviceControlPin
            )
        }
    }
}
