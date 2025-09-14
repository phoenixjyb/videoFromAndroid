package com.example.camcontrol.transport

// Mirrored into module path for build

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
sealed class ControlCommand

@Serializable
@SerialName("setZoomRatio")
data class SetZoomRatio(val value: Float) : ControlCommand()

@Serializable
@SerialName("setAeLock")
data class SetAeLock(val value: Boolean) : ControlCommand()

@Serializable
@SerialName("setAwbLock")
data class SetAwbLock(val value: Boolean) : ControlCommand()

@Serializable
@SerialName("startRecording")
data class StartRecording(val name: String? = null) : ControlCommand()

@Serializable
@SerialName("stopRecording")
object StopRecording : ControlCommand()

@Serializable
@SerialName("setVideoProfile")
data class SetVideoProfile(
    val width: Int,
    val height: Int,
    val fps: Int,
    val highSpeed: Boolean = false
) : ControlCommand()

@Serializable
@SerialName("switchCamera")
data class SwitchCamera(
    // Accepts "back" or "front"; future: cameraId
    val facing: String
) : ControlCommand()

@Serializable
@SerialName("setBitrate")
data class SetBitrate(
    // Bitrate in bits per second
    val bitrate: Int
) : ControlCommand()

@Serializable
@SerialName("requestKeyFrame")
object RequestKeyFrame : ControlCommand()
