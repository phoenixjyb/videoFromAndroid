package com.example.camcontrol.transport

// Mirrored into module path for build

import kotlinx.serialization.Serializable

@Serializable
data class Telemetry(
    val af: Int? = null,
    val ae: Int? = null,
    val iso: Int? = null,
    val expNs: Long? = null,
    val zoom: Float? = null,
    val fps: Float? = null
)

