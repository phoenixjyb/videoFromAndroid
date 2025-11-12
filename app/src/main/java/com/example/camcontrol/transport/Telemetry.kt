package com.example.camcontrol.transport

// Mirrored into module path for build

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class Telemetry(
    // Camera metadata
    val af: Int? = null,
    val ae: Int? = null,
    val iso: Int? = null,
    @SerialName("exp_ns")
    val expNs: Long? = null,
    val zoom: Float? = null,
    
    // Video stream metadata (for viewer)
    @SerialName("frame_number")
    val frameNumber: Int = 0,
    @SerialName("timestamp_ms")
    val timestampMs: Long = System.currentTimeMillis(),
    val fps: Float? = null,
    @SerialName("bitrate_kbps")
    val bitrateKbps: Int = 0,
    val resolution: Resolution? = null,
    val codec: String = "",
    @SerialName("tracking_data")
    val trackingData: TrackingData? = null
)

@Serializable
data class Resolution(
    val width: Int,
    val height: Int
)

@Serializable
data class TrackingData(
    val bbox: BoundingBox? = null,
    val confidence: Float = 0f,
    @SerialName("track_id")
    val trackId: Int? = null
)

@Serializable
data class BoundingBox(
    val x: Float,
    val y: Float,
    val width: Float,
    val height: Float
)

