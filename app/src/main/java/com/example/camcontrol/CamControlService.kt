package com.example.camcontrol

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.graphics.SurfaceTexture
import android.os.Build
import android.os.IBinder
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import androidx.core.app.NotificationCompat
import com.example.camcontrol.camera.Camera2Controller
import com.example.camcontrol.encode.VideoEncoder
import com.example.camcontrol.encode.VideoRecorder
import com.example.camcontrol.transport.*
import kotlinx.coroutines.*
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json

/**
 * Foreground service that keeps the camera + encoder + WebSocket server running
 * while the Activity is backgrounded. Shows a persistent notification.
 */
class CamControlService : Service() {

    private val coroutineErrorHandler = CoroutineExceptionHandler { _, e ->
        Log.e(TAG, "Service coroutine error", e)
        updateNotification("Error: ${e.javaClass.simpleName}")
    }
    private val serviceScope = CoroutineScope(SupervisorJob() + Dispatchers.Main + coroutineErrorHandler)

    private lateinit var cameraController: Camera2Controller
    private lateinit var videoEncoder: VideoEncoder
    private lateinit var controlServer: ControlServer
    private var videoRecorder: VideoRecorder? = null

    private var cameraThread: HandlerThread? = null
    private var cameraHandler: Handler? = null

    private var headlessSurfaceTex: SurfaceTexture? = null
    private var headlessPreviewSurface: Surface? = null

    private val json = Json { classDiscriminator = "cmd"; ignoreUnknownKeys = true }
    private var currentProfile = VideoProfile(1920, 1080, 30, highSpeed = false)
    private val defaultProfile = VideoProfile(1920, 1080, 30, highSpeed = false)
    @Volatile private var currentBitrate: Int? = null
    @Volatile private var currentCodec: String = "h265"
    private val encoderMutex = Mutex()  // Prevent concurrent encoder reconfigurations

    private val telemetryReceiver = object : android.content.BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                "com.example.camcontrol.TELEMETRY" -> {
                    val payload = intent.getStringExtra("payload")
                    if (!payload.isNullOrEmpty()) {
                        Log.d(TAG, "Telemetry intent received (${payload.length} bytes)")
                        serviceScope.launch {
                            try { controlServer.broadcastText(payload) } catch (_: Throwable) {}
                        }
                    }
                }
                "com.example.camcontrol.REQUEST_ENCODER_SURFACE" -> {
                    Log.d(TAG, "REQUEST_ENCODER_SURFACE received; rebroadcasting if ready")
                    broadcastEncoderSurface()
                }
            }
        }
    }

    companion object {
        private const val TAG = "CamControlService"
        private const val CH_ID = "camcontrol_fg"
        private const val NOTIF_ID = 42

        const val ACTION_STOP = "com.example.camcontrol.action.STOP"
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        Log.d(TAG, "🚀 SERVICE onCreate() - Starting service creation")
        createNotifChannel()
        Log.d(TAG, "📢 Notification channel created")
        startForeground(NOTIF_ID, buildNotification("Starting…"))
        Log.d(TAG, "⚡ Foreground service started with notification")
        startBackgroundThread()
        Log.d(TAG, "🧵 Background thread started")
        try {
            Log.d(TAG, "🔧 Starting stack initialization...")
            startStack()
            Log.d(TAG, "✅ Stack initialization completed successfully")
        } catch (t: Throwable) {
            Log.e(TAG, "❌ startStack failed in onCreate", t)
            updateNotification("Startup failed: ${t.javaClass.simpleName}")
            // Avoid crashing the process; stop service gracefully
            stopSelf()
        }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (intent?.action == ACTION_STOP) {
            stopSelf()
            return START_NOT_STICKY
        }
        return START_STICKY
    }

    override fun onDestroy() {
        super.onDestroy()
        Log.i(TAG, "Service destroying")
        serviceScope.cancel()
        try { controlServer.stop() } catch (_: Throwable) {}
        try { videoEncoder.stop() } catch (_: Throwable) {}
        try { cameraController.close() } catch (_: Throwable) {}
        stopBackgroundThread()
        try { headlessPreviewSurface?.release(); headlessSurfaceTex?.release() } catch (_: Throwable) {}
        try { unregisterReceiver(telemetryReceiver) } catch (_: Throwable) {}
    }

    private fun createNotifChannel() {
        if (Build.VERSION.SDK_INT >= 26) {
            val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            val ch = NotificationChannel(CH_ID, "CamControl", NotificationManager.IMPORTANCE_LOW)
            ch.description = "CamControl foreground service"
            nm.createNotificationChannel(ch)
        }
    }

    private fun buildNotification(sub: String): Notification {
        val stopIntent = Intent(this, CamControlService::class.java).apply { action = ACTION_STOP }
        val stopPi = PendingIntent.getService(
            this, 1, stopIntent,
            PendingIntent.FLAG_UPDATE_CURRENT or (if (Build.VERSION.SDK_INT >= 23) PendingIntent.FLAG_IMMUTABLE else 0)
        )
        val openIntent = Intent(this, MainActivity::class.java)
        val openPi = PendingIntent.getActivity(
            this, 2, openIntent,
            PendingIntent.FLAG_UPDATE_CURRENT or (if (Build.VERSION.SDK_INT >= 23) PendingIntent.FLAG_IMMUTABLE else 0)
        )
        return NotificationCompat.Builder(this, CH_ID)
            .setSmallIcon(android.R.drawable.presence_video_online)
            .setContentTitle("CamControl streaming")
            .setContentText(sub)
            .setContentIntent(openPi)
            .setOngoing(true)
            .addAction(0, "Stop", stopPi)
            .build()
    }

    private fun updateNotification(sub: String) {
        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.notify(NOTIF_ID, buildNotification(sub))
    }

    private fun startBackgroundThread() {
        cameraThread = HandlerThread("CameraBackground").also { it.start() }
        cameraHandler = Handler(cameraThread!!.looper)
    }

    private fun stopBackgroundThread() {
        cameraThread?.quitSafely()
        try {
            cameraThread?.join()
        } catch (_: InterruptedException) {}
        cameraThread = null
        cameraHandler = null
    }

    private fun startStack() {
        Log.d(TAG, "🔧 startStack() - Beginning stack initialization")
        
        // Create a headless preview surface so capture continues when no Activity UI
        Log.d(TAG, "🏗️ Creating headless surface texture...")
        headlessSurfaceTex = SurfaceTexture(0).apply { setDefaultBufferSize(currentProfile.width, currentProfile.height) }
        headlessPreviewSurface = Surface(headlessSurfaceTex)
        Log.d(TAG, "✅ Headless surface created: ${currentProfile.width}x${currentProfile.height}")

        Log.d(TAG, "🌐 Creating ControlServer...")
        controlServer = ControlServer(this, serviceScope) { commandString ->
            Log.d(TAG, "📨 WebSocket command received: $commandString")
            try {
                when (val command = json.decodeFromString<ControlCommand>(commandString)) {
                    is SetZoomRatio  -> {
                        Log.d(TAG, "🔍 Zoom command received: ${command.value}, forwarding to MainActivity")
                        // Forward to MainActivity via explicit broadcast intent
                        val intent = Intent("com.example.camcontrol.CAMERA_COMMAND").apply {
                            setPackage(packageName) // Make broadcast explicit
                            putExtra("command", "setZoomRatio")
                            putExtra("value", command.value)
                        }
                        sendBroadcast(intent)
                        Log.d(TAG, "📡 Zoom broadcast sent: action=${intent.action}, command=${intent.getStringExtra("command")}, value=${intent.getFloatExtra("value", 0f)}")
                    }
                    is SetAeLock     -> {
                        Log.d(TAG, "☀️ AE Lock command received: ${command.value}")
                        val intent = Intent("com.example.camcontrol.CAMERA_COMMAND").apply {
                            setPackage(packageName) // Make broadcast explicit
                            putExtra("command", "setAeLock")
                            putExtra("value", command.value)
                        }
                        sendBroadcast(intent)
                        Log.d(TAG, "📡 AE Lock broadcast sent")
                    }
                    is SetAwbLock    -> {
                        Log.d(TAG, "📷 AWB Lock command received: ${command.value}")
                        val intent = Intent("com.example.camcontrol.CAMERA_COMMAND").apply {
                            setPackage(packageName) // Make broadcast explicit
                            putExtra("command", "setAwbLock")
                            putExtra("value", command.value)
                        }
                        sendBroadcast(intent)
                        Log.d(TAG, "📡 AWB Lock broadcast sent")
                    }
                    is StartRecording-> serviceScope.launch { startRecording(command.name) }
                    is StopRecording -> serviceScope.launch { stopRecording() }
                    is SetVideoProfile -> {
                        // Forward to Activity; Activity will restart camera pipeline.
                        val intent = Intent("com.example.camcontrol.CAMERA_COMMAND").apply {
                            setPackage(packageName)
                            putExtra("command", "setVideoProfile")
                            putExtra("width", command.width)
                            putExtra("height", command.height)
                            putExtra("fps", command.fps)
                            putExtra("highSpeed", command.highSpeed)
                        }
                        sendBroadcast(intent)
                        // Optionally reconfigure encoder to match; rebroadcast surface
                        serviceScope.launch(Dispatchers.IO) {
                            encoderMutex.withLock {
                                try {
                                    currentProfile = VideoProfile(command.width, command.height, command.fps, command.highSpeed)
                                    val req = currentBitrate ?: estimateBitrate(currentProfile)
                                    val br = capBitrateForProfile(currentProfile, req)
                                    videoEncoder.stop()
                                    videoEncoder.configure(currentProfile.width, currentProfile.height, currentProfile.fps, br, currentCodec)
                                    videoEncoder.start()
                                    broadcastEncoderSurface()
                                } catch (t: Throwable) {
                                    Log.w(TAG, "Encoder reconfigure failed", t)
                                }
                            }
                        }
                    }
                    is SetBitrate -> {
                        Log.d(TAG, "🎛️ Bitrate change requested: ${command.bitrate}")
                        // Deduplicate and clamp to reasonable range for current profile
                        val applied = capBitrateForProfile(currentProfile, command.bitrate)
                        if (currentBitrate == applied) {
                            Log.d(TAG, "Bitrate unchanged (applied=${applied})")
                            return@ControlServer
                        }
                        currentBitrate = applied
                        // Use dynamic bitrate update (no encoder restart needed)
                        serviceScope.launch(Dispatchers.IO) {
                            encoderMutex.withLock {
                                try {
                                    videoEncoder.setBitrate(applied)
                                    Log.d(TAG, "✅ Bitrate updated dynamically to ${applied / 1_000_000} Mbps")
                                } catch (t: Throwable) {
                                    Log.w(TAG, "Dynamic bitrate update failed, will try full reconfigure", t)
                                    // Fallback: full reconfigure if dynamic update fails
                                    try {
                                        val p = currentProfile
                                        videoEncoder.stop()
                                        videoEncoder.configure(p.width, p.height, p.fps, applied, currentCodec)
                                        videoEncoder.start()
                                        broadcastEncoderSurface()
                                    } catch (t2: Throwable) {
                                        Log.e(TAG, "Bitrate reconfigure failed", t2)
                                    }
                                }
                            }
                        }
                    }
                    is SwitchCamera -> {
                        Log.d(TAG, "📷 Camera switch command received: ${command.facing}")
                        val intent = Intent("com.example.camcontrol.CAMERA_COMMAND").apply {
                            setPackage(packageName) // Make broadcast explicit
                            putExtra("command", "switchCamera")
                            putExtra("facing", command.facing)
                        }
                        sendBroadcast(intent)
                        Log.d(TAG, "📡 Camera switch broadcast sent: action=${intent.action}, command=${intent.getStringExtra("command")}, facing=${intent.getStringExtra("facing")}")
                    }
                    is SetCodec -> {
                        val value = command.codec.lowercase()
                        val mapped = if (value == "h265" || value == "hevc") "h265" else "h264"
                        Log.d(TAG, "🎚️ Codec change requested: ${mapped}")
                        if (currentCodec == mapped) return@ControlServer
                        currentCodec = mapped
                        serviceScope.launch(Dispatchers.IO) {
                            encoderMutex.withLock {
                                try {
                                    val p = currentProfile
                                    val br = currentBitrate ?: estimateBitrate(p)
                                    videoEncoder.stop()
                                    videoEncoder.configure(p.width, p.height, p.fps, br, currentCodec)
                                    videoEncoder.start()
                                    broadcastEncoderSurface()
                                } catch (t: Throwable) {
                                    Log.w(TAG, "Codec reconfigure failed", t)
                                }
                            }
                        }
                    }
                    is RequestKeyFrame -> {
                        Log.d(TAG, "🎬 RequestKeyFrame received")
                        try { videoEncoder.requestKeyFrame() } catch (_: Throwable) {}
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "❌ Failed to parse command: $commandString", e)
            }
        }
        Log.d(TAG, "✅ ControlServer created successfully")

        // Receive telemetry from Activity and forward to WebSocket clients, and handle encoder surface requests
        try {
            val filter = IntentFilter("com.example.camcontrol.TELEMETRY").apply {
                addAction("com.example.camcontrol.REQUEST_ENCODER_SURFACE")
            }
            if (Build.VERSION.SDK_INT >= 33) {
                registerReceiver(telemetryReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
            } else {
                registerReceiver(telemetryReceiver, filter)
            }
            Log.d(TAG, "📡 Telemetry receiver registered in service")
        } catch (t: Throwable) {
            Log.w(TAG, "Failed to register telemetry receiver", t)
        }

        // NOTE: Service now only handles WebSocket server, MainActivity handles camera
        Log.d(TAG, "🎥 Creating VideoEncoder...")
        videoEncoder = VideoEncoder(serviceScope) { data ->
            serviceScope.launch {
                try { controlServer.broadcastVideoData(data) } catch (e: Exception) {
                    Log.w(TAG, "broadcast video error", e)
                }
            }
        }
        Log.d(TAG, "✅ VideoEncoder created")

        // Start encoder and announce its input Surface to the Activity
        try {
            val p = currentProfile
            val br = capBitrateForProfile(p, currentBitrate ?: estimateBitrate(p))
            videoEncoder.configure(p.width, p.height, p.fps, br, currentCodec)
            videoEncoder.start()
            broadcastEncoderSurface()
        } catch (t: Throwable) {
            Log.w(TAG, "Failed to start encoder", t)
        }
        
        Log.d(TAG, "📹 Creating VideoRecorder...")
        videoRecorder = VideoRecorder(this)
        Log.d(TAG, "✅ VideoRecorder created")

        // Don't initialize camera controller in service - let MainActivity handle it
        // cameraController = Camera2Controller(this, cameraHandler!!) { telemetry: Telemetry ->
        //     try {
        //         val payload = json.encodeToString(telemetry)
        //         serviceScope.launch { try { controlServer.broadcastText(payload) } catch (_: Throwable) {} }
        //     } catch (_: Throwable) {}
        // }

        Log.d(TAG, "🚀 Starting ControlServer...")
        controlServer.start()
        Log.d(TAG, "✅ ControlServer started successfully!")
        updateNotification("WebSocket server ready")
        Log.d(TAG, "🎉 startStack() completed successfully - WebSocket server is running")
        // Don't start pipeline in service - let MainActivity handle camera
        // serviceScope.launch(Dispatchers.IO) {
        //     try { startPipeline(currentProfile) } catch (t: Throwable) {
        //         Log.e(TAG, "Initial pipeline error", t)
        //     }
        // }
    }

    private fun broadcastEncoderSurface() {
        val surf = videoEncoder.inputSurface ?: run {
            Log.w(TAG, "Encoder surface unavailable; cannot broadcast")
            return
        }
        val p = currentProfile
        val intent = Intent("com.example.camcontrol.ENCODER_SURFACE").apply {
            setPackage(packageName)
            putExtra("width", p.width)
            putExtra("height", p.height)
            putExtra("fps", p.fps)
            putExtra("highSpeed", p.highSpeed)
            putExtra("surface", surf)
        }
        sendBroadcast(intent)
        Log.d(TAG, "🎬 Encoder surface broadcasted: ${p.width}x${p.height}@${p.fps}")
    }

    private suspend fun startPipeline(profile: VideoProfile) {
        withContext(Dispatchers.IO) {
            updateNotification("Starting ${profile.width}x${profile.height}@${profile.fps}")
            val targetBitrate = estimateBitrate(profile)
            videoEncoder.configure(profile.width, profile.height, profile.fps, targetBitrate, currentCodec)
            videoEncoder.start()
            val encoderSurface = videoEncoder.inputSurface ?: throw IllegalStateException("Encoder input surface unavailable")
            cameraController.openCamera()
            cameraController.startCaptureSession(
                headlessPreviewSurface ?: throw IllegalStateException("Preview surface null"),
                encoderSurface,
                profile.fps,
                profile.highSpeed
            )
            updateNotification("Streaming ${profile.width}x${profile.height}@${profile.fps}")
        }
    }

    private suspend fun restartPipeline(profile: VideoProfile) {
        try { stopRecording() } catch (_: Throwable) {}
        try { videoEncoder.stop() } catch (_: Throwable) {}
        try { cameraController.close() } catch (_: Throwable) {}
        delay(250)
        currentProfile = profile
        headlessSurfaceTex?.setDefaultBufferSize(profile.width, profile.height)
        try {
            startPipeline(profile)
        } catch (e: Exception) {
            Log.w(TAG, "Pipeline failed, fallback", e)
            currentProfile = defaultProfile
            headlessSurfaceTex?.setDefaultBufferSize(defaultProfile.width, defaultProfile.height)
            try { startPipeline(defaultProfile) } catch (ee: Exception) {
                Log.e(TAG, "Fallback pipeline failed", ee)
            }
        }
    }

    private fun startRecording(name: String?) {
        val recorder = videoRecorder ?: return
        val file = recorder.startRecording(name)
        videoEncoder.setMuxerSink(recorder)
        updateNotification("Recording… ${file.name}")
    }

    private fun stopRecording() {
        videoEncoder.setMuxerSink(null)
        val file = videoRecorder?.stopRecording()
        updateNotification("Saved ${file?.name ?: ""}")
    }

    private fun estimateBitrate(p: VideoProfile): Int {
        // Slightly higher bits-per-pixel defaults for clearer Web preview
        val bpp = when {
            p.width >= 3840 -> 0.18f        // 4K30 ≈ 50 Mbps (capped below)
            p.width >= 2560 -> 0.14f
            p.width >= 1920 && p.fps > 60 -> 0.12f
            p.width >= 1920 -> 0.12f        // 1080p30 ≈ 9.3 Mbps
            else -> 0.10f
        }
        var bitrate = (p.width * p.height * p.fps * bpp).toInt()
        
        // H.265 is ~40-50% more efficient than H.264, but we want same visual quality
        // So we actually keep similar bitrate (slightly higher for headroom)
        // This gives better quality than H.264 at same bitrate
        if (currentCodec == "h265") {
            bitrate = (bitrate * 1.1f).toInt()  // 10% boost for H.265 quality headroom
        }
        
        return bitrate.coerceIn(5_000_000, 50_000_000)
    }

    private fun capBitrateForProfile(p: VideoProfile, requested: Int): Int {
        // Conservative caps to avoid encoder instability across devices.
        val max = when {
            p.width >= 3840 -> 45_000_000
            p.width >= 2560 -> 35_000_000
            p.width >= 1920 && p.fps > 60 -> 28_000_000
            p.width >= 1920 -> 20_000_000
            else -> 12_000_000
        }
        val min = 2_000_000
        val applied = requested.coerceIn(min, max)
        if (applied != requested) {
            Log.d(TAG, "Bitrate clamped: requested=${requested} applied=${applied} for ${p.width}x${p.height}@${p.fps}")
        }
        return applied
    }

    data class VideoProfile(val width: Int, val height: Int, val fps: Int, val highSpeed: Boolean)
}
