package com.example.camviewer.ui.screens.video

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
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
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.hilt.navigation.compose.hiltViewModel
import com.example.camviewer.data.model.ConnectionState
import com.example.camviewer.video.VideoSurfaceView
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

@Composable
fun VideoScreen(
    viewModel: VideoViewModel = hiltViewModel()
) {
    val connectionState by viewModel.connectionState.collectAsState()
    val telemetry by viewModel.telemetry.collectAsState()
    val latency by viewModel.latency.collectAsState()
    val scope = rememberCoroutineScope()
    
    var videoSurfaceView by remember { mutableStateOf<VideoSurfaceView?>(null) }
    var tapPosition by remember { mutableStateOf<Offset?>(null) }
    
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
                    detectTapGestures { offset ->
                        // Convert tap to normalized coordinates (0.0-1.0)
                        val normalizedX = offset.x / size.width
                        val normalizedY = offset.y / size.height
                        
                        tapPosition = offset
                        
                        // Send target coordinates
                        viewModel.sendTargetCoordinates(normalizedX, normalizedY)
                        
                        // Clear tap indicator after delay
                        scope.launch {
                            delay(1000)
                            tapPosition = null
                        }
                    }
                }
        )
        
        // Tap indicator (crosshair)
        tapPosition?.let { position ->
            TapCrosshair(position)
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

