package com.example.camviewer.ui.screens.settings

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.camviewer.data.model.AppSettings
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
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SettingsScreen(
    viewModel: SettingsViewModel = hiltViewModel()
) {
    val settings by viewModel.settings.collectAsState()
    
    var cameraUrl by remember { mutableStateOf(settings.cameraUrl) }
    var orinTargetUrl by remember { mutableStateOf(settings.orinTargetUrl) }
    var orinMediaUrl by remember { mutableStateOf(settings.orinMediaUrl) }
    var developerMode by remember { mutableStateOf(settings.developerModeEnabled) }
    var showSaved by remember { mutableStateOf(false) }
    
    // Update local state when settings change
    LaunchedEffect(settings) {
        cameraUrl = settings.cameraUrl
        orinTargetUrl = settings.orinTargetUrl
        orinMediaUrl = settings.orinMediaUrl
        developerMode = settings.developerModeEnabled
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
        
        // Camera Connection Section
        Text(
            text = "Camera Connection",
            style = MaterialTheme.typography.titleLarge
        )
        
        OutlinedTextField(
            value = cameraUrl,
            onValueChange = { cameraUrl = it },
            label = { Text("Camera WebSocket URL") },
            placeholder = { Text("ws://172.16.30.28:9090") },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true
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
            onValueChange = { orinTargetUrl = it },
            label = { Text("Target API URL") },
            placeholder = { Text("http://172.16.30.234:8080") },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true
        )
        
        OutlinedTextField(
            value = orinMediaUrl,
            onValueChange = { orinMediaUrl = it },
            label = { Text("Media API URL") },
            placeholder = { Text("http://172.16.30.234:8081") },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true
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
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Save Button
        Button(
            onClick = {
                viewModel.updateSettings(
                    AppSettings(
                        cameraUrl = cameraUrl,
                        orinTargetUrl = orinTargetUrl,
                        orinMediaUrl = orinMediaUrl,
                        developerModeEnabled = developerMode
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
