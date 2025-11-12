package com.example.camviewer.ui.screens.video

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp

/**
 * Camera Control Overlay
 * 
 * Developer mode UI that appears as a semi-transparent overlay on video stream.
 * Can be collapsed to a compact bar to allow bounding box dragging.
 * Mirrors webUI controls for camera parameters:
 * - Zoom control
 * - Auto Exposure (AE) Lock
 * - Auto White Balance (AWB) Lock  
 * - Camera switch (front/back)
 * - Bitrate adjustment
 * - Codec selection (h264/h265)
 */
@Composable
fun CameraControlOverlay(
    isVisible: Boolean,
    onZoomChange: (Float) -> Unit,
    onAeLockChange: (Boolean) -> Unit,
    onAwbLockChange: (Boolean) -> Unit,
    onCameraSwitch: (String) -> Unit,
    onBitrateChange: (Int) -> Unit,
    onCodecChange: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    var isExpanded by remember { mutableStateOf(false) }
    
    AnimatedVisibility(
        visible = isVisible,
        enter = fadeIn(),
        exit = fadeOut()
    ) {
        Surface(
            modifier = modifier
                .wrapContentHeight()
                .padding(start = 16.dp, end = 8.dp, bottom = 16.dp),
            shape = RoundedCornerShape(12.dp),
            color = MaterialTheme.colorScheme.surface.copy(alpha = 0.5f),
            tonalElevation = 4.dp
        ) {
            Column(
                modifier = Modifier
                    .padding(16.dp)
                    .fillMaxWidth()
            ) {
                // Header with expand/collapse button
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = "Developer Controls",
                        style = MaterialTheme.typography.titleMedium,
                        color = MaterialTheme.colorScheme.primary
                    )
                    IconButton(
                        onClick = { isExpanded = !isExpanded },
                        modifier = Modifier.size(32.dp)
                    ) {
                        Icon(
                            imageVector = if (isExpanded) Icons.Default.KeyboardArrowDown else Icons.Default.KeyboardArrowUp,
                            contentDescription = if (isExpanded) "Collapse" else "Expand",
                            tint = MaterialTheme.colorScheme.primary
                        )
                    }
                }
                
                // Expanded controls
                AnimatedVisibility(visible = isExpanded) {
                    Column {
                        Spacer(modifier = Modifier.height(12.dp))
                        
                        // Zoom Control
                        var zoom by remember { mutableFloatStateOf(1.0f) }
                        Text("Zoom: ${String.format("%.1fx", zoom)}")
                        Slider(
                            value = zoom,
                            onValueChange = { 
                                zoom = it
                                onZoomChange(it)
                            },
                            valueRange = 1.0f..10.0f,
                            steps = 17
                        )
                        
                        Spacer(modifier = Modifier.height(8.dp))
                        
                        // AE Lock
                        var aeLock by remember { mutableStateOf(false) }
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text("AE Lock (Exposure)")
                            Switch(
                                checked = aeLock,
                                onCheckedChange = {
                                    aeLock = it
                                    onAeLockChange(it)
                                }
                            )
                        }
                        
                        Spacer(modifier = Modifier.height(8.dp))
                        
                        // AWB Lock
                        var awbLock by remember { mutableStateOf(false) }
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text("AWB Lock (White Balance)")
                            Switch(
                                checked = awbLock,
                                onCheckedChange = {
                                    awbLock = it
                                    onAwbLockChange(it)
                                }
                            )
                        }
                        
                        Spacer(modifier = Modifier.height(12.dp))
                        
                        // Camera Switch
                        var selectedCamera by remember { mutableStateOf("back") }
                        Text("Camera", style = MaterialTheme.typography.titleSmall)
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.spacedBy(8.dp)
                        ) {
                            FilterChip(
                                selected = selectedCamera == "back",
                                onClick = {
                                    selectedCamera = "back"
                                    onCameraSwitch("back")
                                },
                                label = { Text("Back") },
                                modifier = Modifier.weight(1f)
                            )
                            FilterChip(
                                selected = selectedCamera == "front",
                                onClick = {
                                    selectedCamera = "front"
                                    onCameraSwitch("front")
                                },
                                label = { Text("Front") },
                                modifier = Modifier.weight(1f)
                            )
                        }
                        
                        Spacer(modifier = Modifier.height(12.dp))
                        
                        // Bitrate Selection
                        var selectedBitrate by remember { mutableStateOf(5000000) }
                        Text("Bitrate", style = MaterialTheme.typography.titleSmall)
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.spacedBy(4.dp)
                        ) {
                            listOf(2000000, 5000000, 10000000, 15000000, 20000000).forEach { bitrate ->
                                FilterChip(
                                    selected = selectedBitrate == bitrate,
                                    onClick = {
                                        selectedBitrate = bitrate
                                        onBitrateChange(bitrate)
                                    },
                                    label = { Text("${bitrate / 1000000}M") },
                                    modifier = Modifier.weight(1f)
                                )
                            }
                        }
                        
                        Spacer(modifier = Modifier.height(12.dp))
                        
                        // Codec Selection
                        var selectedCodec by remember { mutableStateOf("h265") }
                        Text("Codec", style = MaterialTheme.typography.titleSmall)
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.spacedBy(8.dp)
                        ) {
                            FilterChip(
                                selected = selectedCodec == "h264",
                                onClick = {
                                    selectedCodec = "h264"
                                    onCodecChange("h264")
                                },
                                label = { Text("H.264") },
                                modifier = Modifier.weight(1f)
                            )
                            FilterChip(
                                selected = selectedCodec == "h265",
                                onClick = {
                                    selectedCodec = "h265"
                                    onCodecChange("h265")
                                },
                                label = { Text("H.265") },
                                modifier = Modifier.weight(1f)
                            )
                        }
                    }
                }
            }
        }
    }
}
