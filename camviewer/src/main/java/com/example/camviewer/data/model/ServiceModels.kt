package com.example.camviewer.data.model

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/**
 * Status of an individual Orin service
 */
@Serializable
data class OrinServiceStatus(
    @SerialName("name")
    val name: String,
    
    @SerialName("running")
    val running: Boolean,
    
    @SerialName("pid")
    val pid: Int? = null,
    
    @SerialName("uptime_seconds")
    val uptimeSeconds: Double? = null,
    
    @SerialName("port")
    val port: Int? = null,  // Nullable for ROS2 nodes without HTTP ports
    
    @SerialName("last_log_lines")
    val lastLogLines: List<String> = emptyList()
)

/**
 * Response from service control operations (start/stop)
 */
@Serializable
data class ServiceControlResponse(
    @SerialName("success")
    val success: Boolean,
    
    @SerialName("message")
    val message: String,
    
    @SerialName("services")
    val services: Map<String, OrinServiceStatus>
)
