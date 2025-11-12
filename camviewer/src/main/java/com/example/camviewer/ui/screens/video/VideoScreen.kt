package com.example.camviewer.ui.screens.video

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Error
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.PointerInputChange
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.input.pointer.positionChange
import androidx.compose.ui.input.pointer.util.VelocityTracker
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.hilt.navigation.compose.hiltViewModel
import com.example.camviewer.data.model.ConnectionState
import com.example.camviewer.video.VideoSurfaceView
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@Composable
fun VideoScreen(
    viewModel: VideoViewModel = hiltViewModel()
) {
    val connectionState by viewModel.connectionState.collectAsState()
    val telemetry by viewModel.telemetry.collectAsState()
    val latency by viewModel.latency.collectAsState()
    val developerModeEnabled by viewModel.developerModeEnabled.collectAsState(initial = false)
    val scope = rememberCoroutineScope()
    
    var videoSurfaceView by remember { mutableStateOf<VideoSurfaceView?>(null) }
    var tapPosition by remember { mutableStateOf<Offset?>(null) }
    var dragStartPosition by remember { mutableStateOf<Offset?>(null) }
    var dragCurrentPosition by remember { mutableStateOf<Offset?>(null) }
    
    // Calculate bounding box from drag positions
    val boundingBox = remember(dragStartPosition, dragCurrentPosition) {
        if (dragStartPosition != null && dragCurrentPosition != null) {
            val start = dragStartPosition!!
            val current = dragCurrentPosition!!
            Rect(
                left = min(start.x, current.x),
                top = min(start.y, current.y),
                right = max(start.x, current.x),
                bottom = max(start.y, current.y)
            )
        } else null
    }
    
    Box(modifier = Modifier.fillMaxSize()) {
        // Video Surface
        AndroidView(
            factory = { context ->
                VideoSurfaceView(context).apply {
                    videoSurfaceView = this
                    
                    setOnSurfaceReadyListener { holder ->
                        viewModel.initializeDecoder(
                            surface = holder.surface,
                            width = 1920,
                            height = 1080
                        )
                    }
                    
                    setOnSurfaceDestroyedListener {
                        // Decoder will be released in ViewModel.onCleared()
                    }
                }
            },
            modifier = Modifier
                .fillMaxSize()
                .pointerInput(Unit) {
                    awaitEachGesture {
                        // Wait for first touch down
                        val down = awaitFirstDown(requireUnconsumed = false)
                        val downPosition = down.position
                        var totalDrag = 0f
                        var isDragging = false
                        
                        // Track movement
                        do {
                            val event = awaitPointerEvent()
                            val dragChange = event.changes.firstOrNull()
                            
                            if (dragChange != null) {
                                val dragAmount = dragChange.position - downPosition
                                totalDrag = kotlin.math.sqrt(
                                    dragAmount.x * dragAmount.x + dragAmount.y * dragAmount.y
                                )
                                
                                // If moved more than 10 pixels, it's a drag
                                if (totalDrag > 10f && !isDragging) {
                                    isDragging = true
                                    // Clear tap position
                                    tapPosition = null
                                    // Start drag
                                    dragStartPosition = downPosition
                                }
                                
                                if (isDragging) {
                                    dragCurrentPosition = dragChange.position
                                }
                            }
                        } while (event.changes.any { it.pressed })
                        
                        // Pointer released
                        if (isDragging) {
                            // Send bounding box
                            val start = dragStartPosition
                            val end = dragCurrentPosition
                            
                            if (start != null && end != null) {
                                // Calculate normalized bounding box (0.0-1.0)
                                val left = (min(start.x, end.x) / size.width).coerceIn(0f, 1f)
                                val top = (min(start.y, end.y) / size.height).coerceIn(0f, 1f)
                                val right = (max(start.x, end.x) / size.width).coerceIn(0f, 1f)
                                val bottom = (max(start.y, end.y) / size.height).coerceIn(0f, 1f)
                                
                                val width = right - left
                                val height = bottom - top
                                
                                // Only send if box has meaningful size (> 1% of screen)
                                if (width > 0.01f && height > 0.01f) {
                                    viewModel.sendTargetROI(left, top, width, height)
                                }
                            }
                            
                            // Clear drag state after delay
                            scope.launch {
                                delay(1000)
                                dragStartPosition = null
                                dragCurrentPosition = null
                            }
                        } else {
                            // It was a tap
                            val normalizedX = (downPosition.x / size.width).coerceIn(0f, 1f)
                            val normalizedY = (downPosition.y / size.height).coerceIn(0f, 1f)
                            
                            tapPosition = downPosition
                            
                            // Send target coordinates
                            viewModel.sendTargetCoordinates(normalizedX, normalizedY)
                            
                            // Clear tap indicator after delay
                            scope.launch {
                                delay(1000)
                                tapPosition = null
                            }
                        }
                    }
                }
        )
        
        // Tap indicator (crosshair)
        tapPosition?.let { position ->
            TapCrosshair(position)
        }
        
        // Bounding box preview during drag
        boundingBox?.let { box ->
            BoundingBoxOverlay(box)
        }
        
        // Telemetry overlay (top-left)
        TelemetryOverlay(
            telemetry = telemetry,
            latency = latency,
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(8.dp)
        )
        
        // Connection status and controls (top-right, compact)
        ConnectionControls(
            connectionState = connectionState,
            onConnect = { viewModel.connect() },
            onDisconnect = { viewModel.disconnect() },
            modifier = Modifier
                .align(Alignment.TopEnd)
                .padding(8.dp)
        )
        
        // Developer mode camera controls overlay (bottom)
        CameraControlOverlay(
            isVisible = developerModeEnabled,
            onZoomChange = { viewModel.setZoom(it) },
            onAeLockChange = { viewModel.setAeLock(it) },
            onAwbLockChange = { viewModel.setAwbLock(it) },
            onCameraSwitch = { viewModel.switchCamera(it) },
            onBitrateChange = { viewModel.setBitrate(it) },
            onCodecChange = { viewModel.setCodec(it) },
            modifier = Modifier
                .align(Alignment.BottomStart)
                .fillMaxWidth(0.85f)  // Leave 15% space on the right
        )
    }
}

@Composable
fun TapCrosshair(position: Offset) {
    Canvas(modifier = Modifier.fillMaxSize()) {
        val crosshairSize = 40.dp.toPx()
        val color = Color.Yellow
        val strokeWidth = 3.dp.toPx()
        
        // Horizontal line
        drawLine(
            color = color,
            start = Offset(position.x - crosshairSize, position.y),
            end = Offset(position.x + crosshairSize, position.y),
            strokeWidth = strokeWidth
        )
        
        // Vertical line
        drawLine(
            color = color,
            start = Offset(position.x, position.y - crosshairSize),
            end = Offset(position.x, position.y + crosshairSize),
            strokeWidth = strokeWidth
        )
        
        // Center circle
        drawCircle(
            color = color,
            radius = 8.dp.toPx(),
            center = position,
            style = androidx.compose.ui.graphics.drawscope.Stroke(width = strokeWidth)
        )
    }
}

@Composable
fun BoundingBoxOverlay(box: Rect) {
    Canvas(modifier = Modifier.fillMaxSize()) {
        val color = Color.Green
        val strokeWidth = 3.dp.toPx()
        val fillColor = Color.Green.copy(alpha = 0.2f)
        
        // Draw filled rectangle
        drawRect(
            color = fillColor,
            topLeft = Offset(box.left, box.top),
            size = androidx.compose.ui.geometry.Size(box.width, box.height)
        )
        
        // Draw border
        drawRect(
            color = color,
            topLeft = Offset(box.left, box.top),
            size = androidx.compose.ui.geometry.Size(box.width, box.height),
            style = androidx.compose.ui.graphics.drawscope.Stroke(width = strokeWidth)
        )
        
        // Draw corner markers (for better visibility)
        val markerSize = 20.dp.toPx()
        
        // Top-left corner
        drawLine(color, Offset(box.left, box.top), Offset(box.left + markerSize, box.top), strokeWidth * 1.5f)
        drawLine(color, Offset(box.left, box.top), Offset(box.left, box.top + markerSize), strokeWidth * 1.5f)
        
        // Top-right corner
        drawLine(color, Offset(box.right, box.top), Offset(box.right - markerSize, box.top), strokeWidth * 1.5f)
        drawLine(color, Offset(box.right, box.top), Offset(box.right, box.top + markerSize), strokeWidth * 1.5f)
        
        // Bottom-left corner
        drawLine(color, Offset(box.left, box.bottom), Offset(box.left + markerSize, box.bottom), strokeWidth * 1.5f)
        drawLine(color, Offset(box.left, box.bottom), Offset(box.left, box.bottom - markerSize), strokeWidth * 1.5f)
        
        // Bottom-right corner
        drawLine(color, Offset(box.right, box.bottom), Offset(box.right - markerSize, box.bottom), strokeWidth * 1.5f)
        drawLine(color, Offset(box.right, box.bottom), Offset(box.right, box.bottom - markerSize), strokeWidth * 1.5f)
    }
}

@Composable
fun TelemetryOverlay(
    telemetry: com.example.camviewer.data.model.Telemetry?,
    latency: Long,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .background(Color.Black.copy(alpha = 0.6f))
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        telemetry?.let { t ->
            Text(
                text = "FPS: ${String.format("%.1f", t.fps)}",
                color = Color.White,
                style = MaterialTheme.typography.bodySmall
            )
            Text(
                text = "Bitrate: ${t.bitrateKbps / 1000} Mbps",
                color = Color.White,
                style = MaterialTheme.typography.bodySmall
            )
            t.resolution?.let { res ->
                Text(
                    text = "Resolution: ${res.width}x${res.height}",
                    color = Color.White,
                    style = MaterialTheme.typography.bodySmall
                )
            }
            Text(
                text = "Codec: ${t.codec}",
                color = Color.White,
                style = MaterialTheme.typography.bodySmall
            )
        }
        
        Text(
            text = "Latency: ${latency}ms",
            color = if (latency < 100) Color.Green else if (latency < 200) Color.Yellow else Color.Red,
            style = MaterialTheme.typography.bodySmall
        )
    }
}

@Composable
fun ConnectionControls(
    connectionState: ConnectionState,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
    modifier: Modifier = Modifier
) {
    Card(
        modifier = modifier,
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surface.copy(alpha = 0.9f)
        )
    ) {
        Row(
            modifier = Modifier.padding(8.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Status indicator with button only
            when (connectionState) {
                is ConnectionState.Disconnected -> {
                    IconButton(
                        onClick = onConnect,
                        modifier = Modifier.size(32.dp)
                    ) {
                        Icon(
                            imageVector = Icons.Default.PlayArrow,
                            contentDescription = "Connect",
                            tint = MaterialTheme.colorScheme.primary,
                            modifier = Modifier.size(20.dp)
                        )
                    }
                }
                is ConnectionState.Connecting -> {
                    CircularProgressIndicator(
                        modifier = Modifier.size(24.dp),
                        strokeWidth = 2.dp
                    )
                }
                is ConnectionState.Connected -> {
                    IconButton(
                        onClick = onDisconnect,
                        modifier = Modifier.size(32.dp)
                    ) {
                        Icon(
                            imageVector = Icons.Default.Stop,
                            contentDescription = "Disconnect",
                            tint = MaterialTheme.colorScheme.error,
                            modifier = Modifier.size(20.dp)
                        )
                    }
                }
                is ConnectionState.Error -> {
                    IconButton(
                        onClick = onConnect,
                        modifier = Modifier.size(32.dp)
                    ) {
                        Icon(
                            imageVector = Icons.Default.Error,
                            contentDescription = "Retry",
                            tint = MaterialTheme.colorScheme.error,
                            modifier = Modifier.size(20.dp)
                        )
                    }
                }
            }
        }
    }
}

