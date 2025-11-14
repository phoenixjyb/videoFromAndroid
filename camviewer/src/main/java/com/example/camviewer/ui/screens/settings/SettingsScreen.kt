package com.example.camviewer.ui.screens.settings

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Visibility
import androidx.compose.material.icons.filled.VisibilityOff
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.text.input.VisualTransformation
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.camviewer.data.model.AppSettings
import com.example.camviewer.data.model.NetworkPreset
import com.example.camviewer.data.repository.SettingsRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class SettingsViewModel @Inject constructor(
    private val settingsRepository: SettingsRepository
) : ViewModel() {
    
    val settings: StateFlow<AppSettings> = settingsRepository.settings
        .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(5000),
            initialValue = AppSettings()
        )
    
    fun updateSettings(settings: AppSettings) {
        viewModelScope.launch {
            settingsRepository.updateSettings(settings)
        }
    }
    
    fun applyNetworkPreset(preset: NetworkPreset) {
        viewModelScope.launch {
            settingsRepository.applyNetworkPreset(preset)
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SettingsScreen(
    viewModel: SettingsViewModel = hiltViewModel()
) {
    val settings by viewModel.settings.collectAsState()
    
    var selectedPreset by remember { mutableStateOf(settings.networkPreset) }
    var showPresetMenu by remember { mutableStateOf(false) }
    var cameraUrl by remember { mutableStateOf(settings.cameraUrl) }
    var orinTargetUrl by remember { mutableStateOf(settings.orinTargetUrl) }
    var orinMediaUrl by remember { mutableStateOf(settings.orinMediaUrl) }
    var developerMode by remember { mutableStateOf(settings.developerModeEnabled) }
    var serviceControlPin by remember { mutableStateOf(settings.serviceControlPin) }
    var showPin by remember { mutableStateOf(false) }
    var showSaved by remember { mutableStateOf(false) }
    
    // Update local state when settings change
    LaunchedEffect(settings) {
        selectedPreset = settings.networkPreset
        cameraUrl = settings.cameraUrl
        orinTargetUrl = settings.orinTargetUrl
        orinMediaUrl = settings.orinMediaUrl
        developerMode = settings.developerModeEnabled
        serviceControlPin = settings.serviceControlPin
    }
    
    Column(
        modifier = Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        Text(
            text = "Settings",
            style = MaterialTheme.typography.headlineMedium
        )
        
        Divider()
        
        // Network Preset Section
        Text(
            text = "Network Preset",
            style = MaterialTheme.typography.titleLarge
        )
        
        ExposedDropdownMenuBox(
            expanded = showPresetMenu,
            onExpandedChange = { showPresetMenu = it }
        ) {
            OutlinedTextField(
                value = selectedPreset.displayName,
                onValueChange = {},
                readOnly = true,
                label = { Text("Network") },
                trailingIcon = { ExposedDropdownMenuDefaults.TrailingIcon(expanded = showPresetMenu) },
                modifier = Modifier
                    .fillMaxWidth()
                    .menuAnchor(),
                colors = ExposedDropdownMenuDefaults.outlinedTextFieldColors()
            )
            
            ExposedDropdownMenu(
                expanded = showPresetMenu,
                onDismissRequest = { showPresetMenu = false }
            ) {
                NetworkPreset.values().forEach { preset ->
                    DropdownMenuItem(
                        text = {
                            Column {
                                Text(preset.displayName)
                                if (preset != NetworkPreset.CUSTOM) {
                                    Text(
                                        text = "Phone: ${preset.phoneIp}, Orin: ${preset.orinIp}",
                                        style = MaterialTheme.typography.bodySmall,
                                        color = MaterialTheme.colorScheme.onSurfaceVariant
                                    )
                                }
                            }
                        },
                        onClick = {
                            selectedPreset = preset
                            viewModel.applyNetworkPreset(preset)
                            showPresetMenu = false
                        }
                    )
                }
            }
        }
        
        Text(
            text = "Select network configuration preset or use Custom for manual entry",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )
        
        Divider()
        
        // Camera Connection Section
        Text(
            text = "Camera Connection",
            style = MaterialTheme.typography.titleLarge
        )
        
        OutlinedTextField(
            value = cameraUrl,
            onValueChange = { 
                cameraUrl = it
                selectedPreset = NetworkPreset.CUSTOM
            },
            label = { Text("Camera WebSocket URL") },
            placeholder = { Text("ws://192.168.100.156:9090") },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true,
            enabled = selectedPreset == NetworkPreset.CUSTOM
        )
        
        Text(
            text = "Enter the WebSocket URL of the CamControl phone",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )
        
        Divider()
        
        // Orin Connection Section
        Text(
            text = "Orin Connection",
            style = MaterialTheme.typography.titleLarge
        )
        
        OutlinedTextField(
            value = orinTargetUrl,
            onValueChange = { 
                orinTargetUrl = it
                selectedPreset = NetworkPreset.CUSTOM
            },
            label = { Text("Target API URL") },
            placeholder = { Text("http://192.168.100.150:8082") },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true,
            enabled = selectedPreset == NetworkPreset.CUSTOM
        )
        
        OutlinedTextField(
            value = orinMediaUrl,
            onValueChange = { 
                orinMediaUrl = it
                selectedPreset = NetworkPreset.CUSTOM
            },
            label = { Text("Media API URL") },
            placeholder = { Text("http://192.168.100.150:8081") },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true,
            enabled = selectedPreset == NetworkPreset.CUSTOM
        )
        
        Text(
            text = "Enter the Orin URLs for target selection and media retrieval",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )
        
        Divider()
        
        // Developer Mode
        Text(
            text = "Advanced",
            style = MaterialTheme.typography.titleLarge
        )
        
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Column(modifier = Modifier.weight(1f)) {
                Text(
                    text = "Developer Mode",
                    style = MaterialTheme.typography.bodyLarge
                )
                Text(
                    text = "Enable camera control panel",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            Switch(
                checked = developerMode,
                onCheckedChange = { developerMode = it }
            )
        }
        
        Divider()
        
        // Security Section
        Text(
            text = "Security",
            style = MaterialTheme.typography.titleLarge
        )
        
        OutlinedTextField(
            value = serviceControlPin,
            onValueChange = { serviceControlPin = it },
            label = { Text("Service Control PIN") },
            placeholder = { Text("Leave empty to disable") },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true,
            visualTransformation = if (showPin) VisualTransformation.None else PasswordVisualTransformation(),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            trailingIcon = {
                IconButton(onClick = { showPin = !showPin }) {
                    Icon(
                        imageVector = if (showPin) Icons.Default.Visibility else Icons.Default.VisibilityOff,
                        contentDescription = if (showPin) "Hide PIN" else "Show PIN"
                    )
                }
            },
            supportingText = {
                Text("PIN required to start/stop Orin services (leave empty for no protection)")
            }
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Save Button
        Button(
            onClick = {
                viewModel.updateSettings(
                    AppSettings(
                        networkPreset = selectedPreset,
                        cameraUrl = cameraUrl,
                        orinTargetUrl = orinTargetUrl,
                        orinMediaUrl = orinMediaUrl,
                        developerModeEnabled = developerMode,
                        serviceControlPin = serviceControlPin
                    )
                )
                showSaved = true
            },
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("Save Settings")
        }
        
        // Saved confirmation
        if (showSaved) {
            LaunchedEffect(Unit) {
                kotlinx.coroutines.delay(2000)
                showSaved = false
            }
            Text(
                text = "✓ Settings saved successfully",
                color = MaterialTheme.colorScheme.primary,
                style = MaterialTheme.typography.bodyMedium
            )
        }
    }
}
