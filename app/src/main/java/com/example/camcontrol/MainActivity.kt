package com.example.camcontrol

// This file mirrors the one at repo root, placed in the proper module path

import android.Manifest
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.hardware.camera2.CameraCharacteristics
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.SurfaceHolder
import android.view.Surface
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.example.camcontrol.camera.Camera2Controller
import com.example.camcontrol.databinding.ActivityMainBinding
import com.example.camcontrol.encode.VideoEncoder
import com.example.camcontrol.encode.VideoRecorder
import com.example.camcontrol.transport.ControlCommand
import com.example.camcontrol.transport.ControlServer
import com.example.camcontrol.transport.SetZoomRatio
import com.example.camcontrol.transport.SetAeLock
import com.example.camcontrol.transport.SetAwbLock
import com.example.camcontrol.transport.Telemetry
import com.example.camcontrol.transport.SetVideoProfile
import com.example.camcontrol.transport.StartRecording
import com.example.camcontrol.transport.StopRecording
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private val activityScope = CoroutineScope(SupervisorJob() + Dispatchers.Main)

    private lateinit var cameraController: Camera2Controller
    private var videoRecorder: VideoRecorder? = null

    private val json = Json { classDiscriminator = "cmd"; ignoreUnknownKeys = true }
    private var currentProfile = VideoProfile(1920, 1080, 30, highSpeed = false)
    private val defaultProfile = VideoProfile(1920, 1080, 30, highSpeed = false)

    private var cameraThread: HandlerThread? = null
    private var cameraHandler: Handler? = null
    private var encoderSurface: Surface? = null

    private val cameraCommandReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            Log.d("CamControl-MainActivity", "BroadcastReceiver.onReceive called - action: ${intent?.action}")
            if (intent?.action == "com.example.camcontrol.CAMERA_COMMAND") {
                val command = intent.getStringExtra("command")
                Log.d("CamControl-MainActivity", "Camera command received: $command")
                
                when (command) {
                    "setZoomRatio" -> {
                        val value = intent.getFloatExtra("value", 1.0f)
                        cameraHandler?.post { 
                            Log.d("CamControl-MainActivity", "Executing zoom: $value")
                            cameraController.setZoomRatio(value) 
                        }
                    }
                    "setAeLock" -> {
                        val value = intent.getBooleanExtra("value", false)
                        cameraHandler?.post { cameraController.setAeLock(value) }
                    }
                    "setAwbLock" -> {
                        val value = intent.getBooleanExtra("value", false)
                        cameraHandler?.post { cameraController.setAwbLock(value) }
                    }
                    "switchCamera" -> {
                        val facing = intent.getStringExtra("facing") ?: "back"
                        val cameraFacing = when (facing.lowercase()) {
                            "front" -> CameraCharacteristics.LENS_FACING_FRONT
                            else -> CameraCharacteristics.LENS_FACING_BACK
                        }
                        Log.d("MainActivity", "📷 Switching camera to: $facing")
                        activityScope.launch {
                            try {
                                cameraController.close()
                                cameraController.setPreferredFacing(cameraFacing)
                                cameraController.openCamera()
                                cameraController.startCaptureSession(
                                    binding.previewView.holder.surface,
                                    encoderSurface,
                                    currentProfile.fps,
                                    currentProfile.highSpeed
                                )
                            } catch (e: Exception) {
                                Log.e("MainActivity", "Error switching camera", e)
                            }
                        }
                    }
                    "setVideoProfile" -> {
                        val w = intent.getIntExtra("width", currentProfile.width)
                        val h = intent.getIntExtra("height", currentProfile.height)
                        val f = intent.getIntExtra("fps", currentProfile.fps)
                        val hs = intent.getBooleanExtra("highSpeed", currentProfile.highSpeed)
                        Log.d("MainActivity", "🎛️ Set video profile: ${w}x${h}@${f} HS=${hs}")
                        val newP = VideoProfile(w, h, f, hs)
                        activityScope.launch {
                            try {
                                restartPipeline(newP)
                            } catch (e: Exception) {
                                Log.e("MainActivity", "Error applying video profile", e)
                            }
                        }
                    }
                }
            } else if (intent?.action == "com.example.camcontrol.ENCODER_SURFACE") {
                // Receive encoder input surface from service
                val surf = if (android.os.Build.VERSION.SDK_INT >= 33) {
                    intent.getParcelableExtra("surface", Surface::class.java)
                } else {
                    @Suppress("DEPRECATION")
                    intent.getParcelableExtra<Surface>("surface")
                }
                val w = intent?.getIntExtra("width", 0)
                val h = intent?.getIntExtra("height", 0)
                val f = intent?.getIntExtra("fps", 0)
                encoderSurface = surf
                Log.d("MainActivity", "🎬 Encoder surface received: ${w}x${h}@${f} (null=${encoderSurface==null})")
                // If preview is active, restart session to include encoder
                activityScope.launch {
                    try {
                        if (binding.previewView.holder.surface.isValid) {
                            restartPipeline(currentProfile)
                        }
                    } catch (e: Exception) {
                        Log.e("MainActivity", "Error restarting pipeline with encoder surface", e)
                    }
                }
            }
        }
    }

    private val requestCameraPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { isGranted: Boolean ->
            if (isGranted) {
                Log.i("PERMISSION", "CAMERA granted")
                startServiceIfReady()
            } else {
                Log.e("PERMISSION", "CAMERA denied")
            }
        }

    private val requestNotifPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { _ ->
            // Regardless of grant result, attempt to start; system may still allow FG notification with low importance
            startServiceIfReady()
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Log.d("CamControl-MainActivity", "Starting CamControl MainActivity onCreate")
        Log.i("CamControl-MainActivity", "Starting CamControl MainActivity onCreate INFO")
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Register broadcast receiver for camera commands from service
        val filter = IntentFilter("com.example.camcontrol.CAMERA_COMMAND").apply {
            addAction("com.example.camcontrol.ENCODER_SURFACE")
        }
        Log.d("CamControl-MainActivity", "Registering broadcast receiver for camera commands")
        if (android.os.Build.VERSION.SDK_INT >= 33) {
            registerReceiver(cameraCommandReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
            Log.d("CamControl-MainActivity", "Broadcast receiver registered with RECEIVER_NOT_EXPORTED API 33+")
        } else {
            registerReceiver(cameraCommandReceiver, filter)
            Log.d("CamControl-MainActivity", "Broadcast receiver registered Legacy API")
        }

        // Prefer running capture in a foreground service so it survives backgrounding
        if (hasCameraPermission()) {
            maybeRequestNotifPermissionThenStart()
        } else {
            requestCameraPermissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    private fun maybeRequestNotifPermissionThenStart() {
        if (android.os.Build.VERSION.SDK_INT >= 33) {
            val granted = ContextCompat.checkSelfPermission(this, android.Manifest.permission.POST_NOTIFICATIONS) == PackageManager.PERMISSION_GRANTED
            if (!granted) {
                requestNotifPermissionLauncher.launch(android.Manifest.permission.POST_NOTIFICATIONS)
                return
            }
        }
        startServiceIfReady()
    }

    private fun startServiceIfReady() {
        Log.i("MainActivity", "Starting CamControlService for video encoding")
        val serviceIntent = Intent(this, CamControlService::class.java)
        startForegroundService(serviceIntent)
        Log.i("MainActivity", "Setting up preview camera")
        setupAndStartCamera()
    }

    private fun startBackgroundThread() {
        cameraThread = HandlerThread("CameraBackground").also { it.start() }
        cameraHandler = Handler(cameraThread!!.looper)
    }

    private fun stopBackgroundThread() {
        cameraThread?.quitSafely()
        try {
            cameraThread?.join()
            cameraThread = null
            cameraHandler = null
        } catch (e: InterruptedException) {
            Log.e("MainActivity", "Error stopping background thread", e)
        }
    }

    private fun setupAndStartCamera() {
        startBackgroundThread()

        // ControlServer is now handled by CamControlService to avoid port conflicts
        Log.i("MainActivity", "WebSocket server will be managed by CamControlService")

        // VideoEncoder is now handled by CamControlService
        // videoEncoder = VideoEncoder(activityScope) { data ->
        //     activityScope.launch { 
        //         try {
        //             controlServer.broadcastVideoData(data)
        //         } catch (e: Exception) {
        //             Log.w("MainActivity", "Failed to broadcast video data", e)
        //         }
        //     }
        // }
        videoRecorder = VideoRecorder(this)

        cameraController = Camera2Controller(this, cameraHandler!!) { telemetry: Telemetry ->
            // Forward telemetry to service for WebSocket broadcast
            try {
                val payload = json.encodeToString(telemetry)
                val intent = Intent("com.example.camcontrol.TELEMETRY").apply {
                    setPackage(packageName)
                    putExtra("payload", payload)
                }
                sendBroadcast(intent)
                Log.v("MainActivity", "Telemetry broadcast sent")
            } catch (t: Throwable) {
                Log.v("MainActivity", "Telemetry send error", t)
            }
        }

        binding.previewView.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                Log.d("MainActivity", "Surface created")
                holder.setFixedSize(currentProfile.width, currentProfile.height)
                // Add a small delay to ensure surface is fully ready
                activityScope.launch(Dispatchers.IO) {
                    try {
                        delay(500) // Wait for surface to stabilize
                        if (holder.surface.isValid) {
                            Log.i("MainActivity", "✅ Surface valid, starting pipeline...")
                            startPipeline(currentProfile)
                        } else {
                            Log.w("MainActivity", "❌ Surface not valid after delay")
                        }
                    } catch (e: Exception) {
                        Log.e("MainActivity", "Initial pipeline start failed", e)
                    }
                }
            }
            override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {}
            override fun surfaceDestroyed(holder: SurfaceHolder) {
                Log.d("MainActivity", "Surface destroyed")
            }
        })

        Log.i("MainActivity", "✅ MainActivity camera setup complete")
    }

    private suspend fun startPipeline(profile: VideoProfile) {
        kotlinx.coroutines.withContext(Dispatchers.IO) {
            Log.i("MainActivity", "🎬 Starting pipeline: ${profile.width}x${profile.height}@${profile.fps}")

            // Validate surface before proceeding
            val surface = binding.previewView.holder.surface
            if (!surface.isValid) {
                throw IllegalArgumentException("Preview surface is not valid")
            }

            // Video encoding is now handled by CamControlService
            Log.i("MainActivity", "📷 Opening camera for preview/encode...")
            cameraController.openCamera()
            Log.i("MainActivity", "🎥 Starting capture session for preview...")
            cameraController.startCaptureSession(surface, encoderSurface, profile.fps, profile.highSpeed)
            Log.i("MainActivity", "✅ Pipeline started successfully!")
        }
    }

    private suspend fun restartPipeline(profile: VideoProfile) {
        try { stopRecording() } catch (_: Throwable) {}
        try { cameraController.close() } catch (_: Throwable) {}
        // Give the camera stack a moment to settle before re-opening (prevents crashes on some devices)
        kotlinx.coroutines.delay(250)
        currentProfile = profile
        runOnUiThread { binding.previewView.holder.setFixedSize(profile.width, profile.height) }
        try {
            startPipeline(profile)
        } catch (e: Exception) {
            Log.w("MainActivity", "Pipeline start failed for ${profile.width}x${profile.height}@${profile.fps} (HS=${profile.highSpeed}); falling back", e)
            currentProfile = defaultProfile
            runOnUiThread { binding.previewView.holder.setFixedSize(defaultProfile.width, defaultProfile.height) }
            try {
                startPipeline(defaultProfile)
            } catch (ee: Exception) {
                Log.e("MainActivity", "Fallback pipeline start failed", ee)
                // Do not rethrow; keep app alive and allow further attempts
            }
        }
    }

    private fun estimateBitrate(p: VideoProfile): Int {
        val bpp = when {
            p.width >= 3840 -> 0.12f
            p.width >= 2560 -> 0.10f
            p.width >= 1920 && p.fps > 60 -> 0.10f
            p.width >= 1920 -> 0.08f
            else -> 0.07f
        }
        val bitrate = (p.width * p.height * p.fps * bpp).toInt()
        return bitrate.coerceIn(3_000_000, 35_000_000)
    }

    data class VideoProfile(val width: Int, val height: Int, val fps: Int, val highSpeed: Boolean)

    private fun hasCameraPermission() = ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED

    private fun startRecording(name: String?) {
        Log.i("MainActivity", "Recording commands are now handled by CamControlService")
        // TODO: Send command to service to start recording
    }

    private fun stopRecording() {
        Log.i("MainActivity", "Recording commands are now handled by CamControlService")
        // TODO: Send command to service to stop recording
    }

    override fun onDestroy() {
        super.onDestroy()
        activityScope.cancel()
        // Unregister broadcast receiver
        try { unregisterReceiver(cameraCommandReceiver) } catch (_: Throwable) {}
        // Do not stop service-managed resources here
        try { cameraController.close() } catch (_: Throwable) {}
        stopBackgroundThread()
    }
}
