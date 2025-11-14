package com.example.camviewer.ui.orin

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.compose.runtime.rememberCoroutineScope
import com.example.camviewer.data.model.OrinServiceStatus
import kotlinx.coroutines.launch

/**
 * Orin services control screen
 * Allows remote start/stop of Orin services and status monitoring
 */
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun OrinControlScreen(
    viewModel: OrinControlViewModel = hiltViewModel()
) {
    val uiState by viewModel.uiState.collectAsState()
    val scope = rememberCoroutineScope()
    var showPinDialog by remember { mutableStateOf(false) }
    var pendingAction by remember { mutableStateOf<PinAction?>(null) }
    
    // PIN dialog
    if (showPinDialog && pendingAction != null) {
        PinDialog(
            action = pendingAction!!,
            onDismiss = { 
                showPinDialog = false
                pendingAction = null
            },
            onConfirm = { pin ->
                showPinDialog = false
                when (pendingAction) {
                    PinAction.START -> viewModel.startServices(pin)
                    PinAction.STOP -> viewModel.stopServices(pin)
                    null -> {}
                }
                pendingAction = null
            }
        )
    }
    
    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Orin Services") },
                actions = {
                    // Auto-refresh toggle
                    IconButton(onClick = { viewModel.toggleAutoRefresh() }) {
                        Icon(
                            imageVector = if (uiState.autoRefreshEnabled) {
                                Icons.Default.Refresh
                            } else {
                                Icons.Default.PlayArrow
                            },
                            contentDescription = if (uiState.autoRefreshEnabled) {
                                "Disable auto-refresh"
                            } else {
                                "Enable auto-refresh"
                            },
                            tint = if (uiState.autoRefreshEnabled) {
                                MaterialTheme.colorScheme.primary
                            } else {
                                MaterialTheme.colorScheme.onSurface
                            }
                        )
                    }
                    
                    // Manual refresh
                    IconButton(
                        onClick = { viewModel.refreshStatus() },
                        enabled = !uiState.isLoading
                    ) {
                        Icon(
                            imageVector = Icons.Default.Refresh,
                            contentDescription = "Refresh status"
                        )
                    }
                }
            )
        }
    ) { padding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .verticalScroll(rememberScrollState())
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            // Control buttons
            ControlButtonsSection(
                onStartAll = { 
                    scope.launch {
                        if (viewModel.requiresPin()) {
                            pendingAction = PinAction.START
                            showPinDialog = true
                        } else {
                            viewModel.startServices()
                        }
                    }
                },
                onStopAll = { 
                    scope.launch {
                        if (viewModel.requiresPin()) {
                            pendingAction = PinAction.STOP
                            showPinDialog = true
                        } else {
                            viewModel.stopServices()
                        }
                    }
                },
                isLoading = uiState.isLoading,
                allRunning = viewModel.areAllServicesRunning()
            )

            // Loading indicator
            if (uiState.isLoading) {
                LinearProgressIndicator(
                    modifier = Modifier.fillMaxWidth()
                )
            }

            // Error message
            uiState.error?.let { error ->
                ErrorCard(
                    message = error,
                    onDismiss = { viewModel.clearError() }
                )
            }

            // Last update time
            if (uiState.lastUpdateTime > 0) {
                LastUpdateInfo(timestamp = uiState.lastUpdateTime)
            }

            // Services status cards
            if (uiState.services.isNotEmpty()) {
                Text(
                    text = "Service Status",
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )

                uiState.services.forEach { (serviceId, status) ->
                    ServiceStatusCard(
                        serviceId = serviceId,
                        status = status
                    )
                }
            } else if (!uiState.isLoading && uiState.error == null) {
                EmptyStateCard()
            }
        }
    }
}

/**
 * Control buttons section (Start All / Stop All)
 */
@Composable
private fun ControlButtonsSection(
    onStartAll: () -> Unit,
    onStopAll: () -> Unit,
    isLoading: Boolean,
    allRunning: Boolean
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        // Start All button
        Button(
            onClick = onStartAll,
            modifier = Modifier.weight(1f),
            enabled = !isLoading,
            colors = ButtonDefaults.buttonColors(
                containerColor = MaterialTheme.colorScheme.primary
            )
        ) {
            Icon(
                imageVector = Icons.Default.PlayArrow,
                contentDescription = null,
                modifier = Modifier.size(20.dp)
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text("Start All")
        }

        // Stop All button
        Button(
            onClick = onStopAll,
            modifier = Modifier.weight(1f),
            enabled = !isLoading && allRunning,
            colors = ButtonDefaults.buttonColors(
                containerColor = MaterialTheme.colorScheme.error
            )
        ) {
            Icon(
                imageVector = Icons.Default.Close,
                contentDescription = null,
                modifier = Modifier.size(20.dp)
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text("Stop All")
        }
    }
}

/**
 * Service status card showing individual service details
 */
@Composable
private fun ServiceStatusCard(
    serviceId: String,
    status: OrinServiceStatus
) {
    var expanded by remember { mutableStateOf(false) }
    
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = if (status.running) {
                MaterialTheme.colorScheme.primaryContainer.copy(alpha = 0.3f)
            } else {
                MaterialTheme.colorScheme.errorContainer.copy(alpha = 0.3f)
            }
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            // Header row
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(12.dp)
                ) {
                    // Status indicator
                    Box(
                        modifier = Modifier
                            .size(12.dp)
                            .background(
                                color = if (status.running) Color.Green else Color.Red,
                                shape = CircleShape
                            )
                    )
                    
                    // Service name
                    Text(
                        text = status.name,
                        style = MaterialTheme.typography.titleMedium,
                        fontWeight = FontWeight.Bold
                    )
                }

                // Status text
                Text(
                    text = if (status.running) "Running" else "Stopped",
                    style = MaterialTheme.typography.bodyMedium,
                    color = if (status.running) {
                        MaterialTheme.colorScheme.primary
                    } else {
                        MaterialTheme.colorScheme.error
                    }
                )
            }

            Spacer(modifier = Modifier.height(8.dp))

            // Details
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                // Left side: Type indicator
                Row(
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    status.port?.let { port ->
                        // HTTP service with port
                        Surface(
                            color = MaterialTheme.colorScheme.secondaryContainer,
                            shape = MaterialTheme.shapes.small
                        ) {
                            Text(
                                text = "Port: $port",
                                style = MaterialTheme.typography.labelSmall,
                                modifier = Modifier.padding(horizontal = 8.dp, vertical = 4.dp)
                            )
                        }
                    } ?: run {
                        // ROS2 node without port
                        Surface(
                            color = MaterialTheme.colorScheme.tertiaryContainer,
                            shape = MaterialTheme.shapes.small
                        ) {
                            Row(
                                horizontalArrangement = Arrangement.spacedBy(4.dp),
                                verticalAlignment = Alignment.CenterVertically,
                                modifier = Modifier.padding(horizontal = 8.dp, vertical = 4.dp)
                            ) {
                                Icon(
                                    imageVector = Icons.Default.Settings,
                                    contentDescription = null,
                                    modifier = Modifier.size(12.dp),
                                    tint = MaterialTheme.colorScheme.onTertiaryContainer
                                )
                                Text(
                                    text = "ROS2 Node",
                                    style = MaterialTheme.typography.labelSmall,
                                    color = MaterialTheme.colorScheme.onTertiaryContainer
                                )
                            }
                        }
                    }
                    
                    status.pid?.let { pid ->
                        Surface(
                            color = MaterialTheme.colorScheme.surfaceVariant,
                            shape = MaterialTheme.shapes.small
                        ) {
                            Text(
                                text = "PID: $pid",
                                style = MaterialTheme.typography.labelSmall,
                                modifier = Modifier.padding(horizontal = 8.dp, vertical = 4.dp)
                            )
                        }
                    }
                }
                
                // Right side: Uptime
                status.uptimeSeconds?.let { uptime ->
                    Text(
                        text = formatUptime(uptime),
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant
                    )
                }
            }

            // Logs section (expandable)
            if (status.lastLogLines.isNotEmpty()) {
                Spacer(modifier = Modifier.height(12.dp))
                
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = "Recent Logs",
                        style = MaterialTheme.typography.labelMedium,
                        fontWeight = FontWeight.SemiBold
                    )
                    
                    IconButton(onClick = { expanded = !expanded }) {
                        Icon(
                            imageVector = if (expanded) {
                                Icons.Default.KeyboardArrowUp
                            } else {
                                Icons.Default.KeyboardArrowDown
                            },
                            contentDescription = if (expanded) "Collapse" else "Expand"
                        )
                    }
                }
                
                if (expanded) {
                    Surface(
                        modifier = Modifier.fillMaxWidth(),
                        color = MaterialTheme.colorScheme.surface,
                        shape = MaterialTheme.shapes.small
                    ) {
                        Column(
                            modifier = Modifier.padding(8.dp)
                        ) {
                            status.lastLogLines.forEach { line ->
                                Text(
                                    text = line,
                                    style = MaterialTheme.typography.bodySmall,
                                    fontFamily = FontFamily.Monospace,
                                    modifier = Modifier.padding(vertical = 2.dp)
                                )
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * Error message card
 */
@Composable
private fun ErrorCard(
    message: String,
    onDismiss: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.errorContainer
        )
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                modifier = Modifier.weight(1f)
            ) {
                Icon(
                    imageVector = Icons.Default.Warning,
                    contentDescription = null,
                    tint = MaterialTheme.colorScheme.error
                )
                Text(
                    text = message,
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.onErrorContainer
                )
            }
            
            IconButton(onClick = onDismiss) {
                Icon(
                    imageVector = Icons.Default.Close,
                    contentDescription = "Dismiss",
                    tint = MaterialTheme.colorScheme.onErrorContainer
                )
            }
        }
    }
}

/**
 * Last update timestamp
 */
@Composable
private fun LastUpdateInfo(timestamp: Long) {
    Text(
        text = "Last updated: ${formatTimestamp(timestamp)}",
        style = MaterialTheme.typography.bodySmall,
        color = MaterialTheme.colorScheme.onSurfaceVariant
    )
}

/**
 * Empty state when no services data
 */
@Composable
private fun EmptyStateCard() {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(32.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Icon(
                imageVector = Icons.Default.Settings,
                contentDescription = null,
                modifier = Modifier.size(48.dp),
                tint = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Text(
                text = "No service data",
                style = MaterialTheme.typography.titleMedium
            )
            Text(
                text = "Tap refresh to check service status",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
        }
    }
}

/**
 * Format uptime seconds to human readable string
 */
private fun formatUptime(seconds: Double): String {
    val totalSeconds = seconds.toInt()
    val hours = totalSeconds / 3600
    val minutes = (totalSeconds % 3600) / 60
    val secs = totalSeconds % 60
    
    return when {
        hours > 0 -> "${hours}h ${minutes}m"
        minutes > 0 -> "${minutes}m ${secs}s"
        else -> "${secs}s"
    }
}

/**
 * Format timestamp to human readable string
 */
private fun formatTimestamp(timestamp: Long): String {
    val now = System.currentTimeMillis()
    val diff = now - timestamp
    val seconds = diff / 1000
    
    return when {
        seconds < 60 -> "just now"
        seconds < 3600 -> "${seconds / 60}m ago"
        else -> "${seconds / 3600}h ago"
    }
}

/**
 * Action type for PIN dialog
 */
enum class PinAction {
    START,
    STOP
}

/**
 * PIN confirmation dialog
 */
@Composable
private fun PinDialog(
    action: PinAction,
    onDismiss: () -> Unit,
    onConfirm: (String) -> Unit
) {
    var pinInput by remember { mutableStateOf("") }
    var pinError by remember { mutableStateOf(false) }
    
    AlertDialog(
        onDismissRequest = onDismiss,
        icon = {
            Icon(
                imageVector = Icons.Default.Lock,
                contentDescription = null
            )
        },
        title = {
            Text(
                text = when (action) {
                    PinAction.START -> "Start Services"
                    PinAction.STOP -> "Stop Services"
                }
            )
        },
        text = {
            Column(
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Text(
                    text = "Enter PIN to ${action.name.lowercase()} Orin services",
                    style = MaterialTheme.typography.bodyMedium
                )
                
                OutlinedTextField(
                    value = pinInput,
                    onValueChange = {
                        pinInput = it
                        pinError = false
                    },
                    label = { Text("PIN") },
                    visualTransformation = PasswordVisualTransformation(),
                    keyboardOptions = KeyboardOptions(
                        keyboardType = KeyboardType.Number
                    ),
                    singleLine = true,
                    isError = pinError,
                    supportingText = if (pinError) {
                        { Text("PIN is required") }
                    } else null,
                    modifier = Modifier.fillMaxWidth()
                )
            }
        },
        confirmButton = {
            TextButton(
                onClick = {
                    if (pinInput.isNotBlank()) {
                        onConfirm(pinInput)
                    } else {
                        pinError = true
                    }
                }
            ) {
                Text("Confirm")
            }
        },
        dismissButton = {
            TextButton(onClick = onDismiss) {
                Text("Cancel")
            }
        }
    )
}
