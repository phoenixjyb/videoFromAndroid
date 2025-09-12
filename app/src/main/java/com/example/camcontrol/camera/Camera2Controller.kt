package com.example.camcontrol.camera

// Mirrored into module path for build

import android.annotation.SuppressLint
import android.content.Context
import android.hardware.camera2.*
import android.os.Handler
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import com.example.camcontrol.transport.Telemetry
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException

class Camera2Controller(
    context: Context,
    private val backgroundHandler: Handler,
    private val onTelemetry: (Telemetry) -> Unit = {}
) {

    private val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var repeatingRequestBuilder: CaptureRequest.Builder? = null
    var zoomRange: android.util.Range<Float>? = null
        private set
    @Volatile private var preferredFacing: Int = CameraCharacteristics.LENS_FACING_BACK

    companion object {
        private const val TAG = "Camera2Controller"
    }

    fun setPreferredFacing(facing: Int) {
        preferredFacing = facing
    }

    @SuppressLint("MissingPermission")
    suspend fun openCamera() {
        val cameraId = when (preferredFacing) {
            CameraCharacteristics.LENS_FACING_FRONT -> getFrontCameraId()
            else -> getBackCameraId()
        } ?: throw CameraAccessException(CameraAccessException.CAMERA_DISABLED, "No matching camera found")
        Log.d(TAG, "Opening camera: $cameraId")

        cameraDevice = suspendCancellableCoroutine { continuation ->
            val stateCallback = object : CameraDevice.StateCallback() {
                override fun onOpened(camera: CameraDevice) {
                    Log.d(TAG, "Camera ${camera.id} opened.")
                    if (continuation.isActive) continuation.resume(camera)
                }
                override fun onDisconnected(camera: CameraDevice) {
                    Log.w(TAG, "Camera ${camera.id} disconnected.")
                    camera.close(); cameraDevice = null
                }
                override fun onError(camera: CameraDevice, error: Int) {
                    val errorMessage = "Camera ${camera.id} error: $error"
                    Log.e(TAG, errorMessage)
                    if (continuation.isActive) continuation.resumeWithException(CameraAccessException(error, errorMessage))
                    cameraDevice = null
                }
            }
            try {
                cameraManager.openCamera(cameraId, stateCallback, backgroundHandler)
            } catch (e: CameraAccessException) {
                continuation.resumeWithException(e)
            }
            continuation.invokeOnCancellation { close() }
        }

        val characteristics = cameraManager.getCameraCharacteristics(cameraDevice!!.id)
        zoomRange = characteristics.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)
        Log.i(TAG, "Camera zoom range: $zoomRange")
    }

    suspend fun startCaptureSession(previewSurface: Surface, encoderSurface: Surface?, fps: Int = 30, highSpeed: Boolean = false) {
        val device = cameraDevice ?: throw IllegalStateException("Camera not open")
        Log.d(TAG, "Starting capture session.")

        captureSession = suspendCancellableCoroutine { continuation ->
            val stateCallback = object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(session: CameraCaptureSession) {
                    Log.d(TAG, "Capture session configured.")
                    if (continuation.isActive) continuation.resume(session)
                }
                override fun onConfigureFailed(session: CameraCaptureSession) {
                    val errorMessage = "Capture session configuration failed"
                    Log.e(TAG, errorMessage)
                    if (continuation.isActive) continuation.resumeWithException(RuntimeException(errorMessage))
                }
            }
            val surfaces = mutableListOf(previewSurface)
            encoderSurface?.let { surfaces.add(it) }
            
            if (highSpeed) {
                device.createConstrainedHighSpeedCaptureSession(surfaces, stateCallback, backgroundHandler)
            } else {
                device.createCaptureSession(surfaces, stateCallback, backgroundHandler)
            }
        }

        startRepeatingRequest(previewSurface, encoderSurface, fps, highSpeed)
    }

    private fun startRepeatingRequest(previewSurface: Surface, encoderSurface: Surface?, fps: Int, highSpeed: Boolean) {
        val session = captureSession ?: throw IllegalStateException("Session not started")
        val device = cameraDevice ?: throw IllegalStateException("Camera not open")

        try {
            repeatingRequestBuilder = device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                addTarget(previewSurface)
                encoderSurface?.let { addTarget(it) }
                set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
                set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
                set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_AUTO)
                set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, android.util.Range(fps, fps))
                set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_ON)
                set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
            }
            val callback = object : CameraCaptureSession.CaptureCallback() {
                private var lastTsNs: Long = 0L
                private var lastEmitMs: Long = 0L
                private var fpsSmoothed: Float? = null
                override fun onCaptureCompleted(session: CameraCaptureSession, request: CaptureRequest, result: TotalCaptureResult) {
                    val nowMs = SystemClock.elapsedRealtime()
                    if (nowMs - lastEmitMs < 100) return
                    lastEmitMs = nowMs
                    val ts = result.get(CaptureResult.SENSOR_TIMESTAMP) ?: 0L
                    if (lastTsNs != 0L && ts > lastTsNs) {
                        val dt = ts - lastTsNs
                        val instFps = 1_000_000_000.0f / dt.toFloat()
                        fpsSmoothed = if (fpsSmoothed == null) instFps else (0.8f * fpsSmoothed!! + 0.2f * instFps)
                    }
                    lastTsNs = ts
                    val telemetry = Telemetry(
                        af = result.get(CaptureResult.CONTROL_AF_STATE),
                        ae = result.get(CaptureResult.CONTROL_AE_STATE),
                        iso = result.get(CaptureResult.SENSOR_SENSITIVITY),
                        expNs = result.get(CaptureResult.SENSOR_EXPOSURE_TIME),
                        zoom = result.get(CaptureResult.CONTROL_ZOOM_RATIO) ?: zoomRange?.lower ?: 1.0f,
                        fps = fpsSmoothed
                    )
                    try { onTelemetry(telemetry) } catch (t: Throwable) { Log.w(TAG, "Telemetry callback error", t) }
                }
            }
            if (highSpeed) {
                val hsSession = session as? CameraConstrainedHighSpeedCaptureSession
                val request = repeatingRequestBuilder!!.build()
                val burst = hsSession?.createHighSpeedRequestList(request)
                if (burst != null) {
                    session.setRepeatingBurst(burst, callback, backgroundHandler)
                } else {
                    session.setRepeatingRequest(request, callback, backgroundHandler)
                }
            } else {
                session.setRepeatingRequest(repeatingRequestBuilder!!.build(), callback, backgroundHandler)
            }
            Log.d(TAG, "Repeating request started.")
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Failed to start repeating request", e)
        }
    }

    fun setZoomRatio(zoom: Float) {
        Log.d(TAG, "🔍 setZoomRatio called: $zoom")
        val range = zoomRange ?: run {
            Log.w(TAG, "🔍 Zoom range is null")
            return
        }
        val builder = repeatingRequestBuilder ?: run {
            Log.w(TAG, "🔍 Repeating request builder is null")
            return
        }
        val session = captureSession ?: run {
            Log.w(TAG, "🔍 Capture session is null")
            return
        }
        val clampedZoom = zoom.coerceIn(range.lower, range.upper)
        Log.d(TAG, "🔍 Setting zoom ratio: $clampedZoom (range: ${range.lower}-${range.upper})")
        builder.set(CaptureRequest.CONTROL_ZOOM_RATIO, clampedZoom)
        try {
            session.setRepeatingRequest(builder.build(), null, backgroundHandler)
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Failed to update zoom", e)
        } catch (e: IllegalStateException) {
            Log.e(TAG, "Camera session is closed, cannot update zoom", e)
        }
    }

    fun setAeLock(lock: Boolean) {
        val builder = repeatingRequestBuilder ?: return
        val session = captureSession ?: return
        builder.set(CaptureRequest.CONTROL_AE_LOCK, lock)
        try {
            session.setRepeatingRequest(builder.build(), null, backgroundHandler)
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Failed to update AE lock", e)
        } catch (e: IllegalStateException) {
            Log.e(TAG, "Camera session is closed, cannot update AE lock", e)
        }
    }

    fun setAwbLock(lock: Boolean) {
        val builder = repeatingRequestBuilder ?: return
        val session = captureSession ?: return
        builder.set(CaptureRequest.CONTROL_AWB_LOCK, lock)
        try {
            session.setRepeatingRequest(builder.build(), null, backgroundHandler)
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Failed to update AWB lock", e)
        } catch (e: IllegalStateException) {
            Log.e(TAG, "Camera session is closed, cannot update AWB lock", e)
        }
    }

    fun close() {
        try { captureSession?.close(); captureSession = null } catch (e: Exception) { Log.e(TAG, "Error closing capture session", e) }
        try { cameraDevice?.close(); cameraDevice = null } catch (e: Exception) { Log.e(TAG, "Error closing camera device", e) }
        Log.d(TAG, "Camera closed.")
    }

    private fun getBackCameraId(): String? {
        if (cameraManager.cameraIdList.contains("0")) {
            val ch = cameraManager.getCameraCharacteristics("0")
            if (ch.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK) return "0"
        }
        var fallback: String? = null
        for (cameraId in cameraManager.cameraIdList) {
            val ch = cameraManager.getCameraCharacteristics(cameraId)
            val facing = ch.get(CameraCharacteristics.LENS_FACING)
            if (facing == CameraCharacteristics.LENS_FACING_BACK) {
                if (fallback == null) fallback = cameraId
                val caps = ch.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: intArrayOf()
                if (caps.contains(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA)) {
                    return cameraId
                }
            }
        }
        return fallback
    }

    private fun getFrontCameraId(): String? {
        // Prefer a logical camera if available, otherwise first front camera
        var fallback: String? = null
        for (cameraId in cameraManager.cameraIdList) {
            val ch = cameraManager.getCameraCharacteristics(cameraId)
            val facing = ch.get(CameraCharacteristics.LENS_FACING)
            if (facing == CameraCharacteristics.LENS_FACING_FRONT) {
                if (fallback == null) fallback = cameraId
                val caps = ch.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: intArrayOf()
                if (caps.contains(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA)) {
                    return cameraId
                }
            }
        }
        return fallback
    }
}
