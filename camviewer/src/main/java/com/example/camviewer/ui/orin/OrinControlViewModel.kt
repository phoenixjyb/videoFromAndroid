package com.example.camviewer.ui.orin

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.camviewer.data.model.OrinServiceStatus
import com.example.camviewer.data.repository.OrinServiceRepository
import com.example.camviewer.data.repository.SettingsRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

/**
 * UI state for Orin services control screen
 */
data class OrinServicesUiState(
    val services: Map<String, OrinServiceStatus> = emptyMap(),
    val isLoading: Boolean = false,
    val error: String? = null,
    val lastUpdateTime: Long = 0L,
    val autoRefreshEnabled: Boolean = false
)

/**
 * ViewModel for controlling Orin services remotely
 */
@HiltViewModel
class OrinControlViewModel @Inject constructor(
    private val orinServiceRepository: OrinServiceRepository,
    private val settingsRepository: SettingsRepository
) : ViewModel() {

    companion object {
        private const val TAG = "OrinControlVM"
        private const val AUTO_REFRESH_INTERVAL_MS = 5000L // 5 seconds
    }

    private val _uiState = MutableStateFlow(OrinServicesUiState())
    val uiState: StateFlow<OrinServicesUiState> = _uiState.asStateFlow()

    init {
        // Observe repository state changes
        viewModelScope.launch {
            combine(
                orinServiceRepository.servicesStatus,
                orinServiceRepository.isLoading,
                orinServiceRepository.error
            ) { services, loading, error ->
                Triple(services, loading, error)
            }.collect { (services, loading, error) ->
                _uiState.update { currentState ->
                    currentState.copy(
                        services = services,
                        isLoading = loading,
                        error = error,
                        lastUpdateTime = if (services.isNotEmpty()) System.currentTimeMillis() else currentState.lastUpdateTime
                    )
                }
            }
        }
    }

    /**
     * Refresh service status from Orin
     */
    fun refreshStatus() {
        viewModelScope.launch {
            Log.d(TAG, "Refreshing service status")
            orinServiceRepository.fetchStatus()
        }
    }

    /**
     * Check if PIN is required for service control
     */
    suspend fun requiresPin(): Boolean {
        val settings = settingsRepository.settings.first()
        return settings.serviceControlPin.isNotEmpty()
    }

    /**
     * Start all Orin services (with optional PIN validation)
     */
    fun startServices(enteredPin: String? = null) {
        viewModelScope.launch {
            // Validate PIN if required
            val settings = settingsRepository.settings.first()
            if (settings.serviceControlPin.isNotEmpty()) {
                if (enteredPin != settings.serviceControlPin) {
                    _uiState.update { it.copy(error = "Incorrect PIN") }
                    return@launch
                }
            }
            
            Log.d(TAG, "Starting all services")
            val result = orinServiceRepository.startServices()
            
            result.onSuccess { response ->
                Log.d(TAG, "Services started successfully: ${response.message}")
                // Wait a bit and refresh to show updated status
                delay(2000)
                refreshStatus()
            }.onFailure { error ->
                Log.e(TAG, "Failed to start services: ${error.message}")
            }
        }
    }

    /**
     * Stop all Orin services (with optional PIN validation)
     */
    fun stopServices(enteredPin: String? = null) {
        viewModelScope.launch {
            // Validate PIN if required
            val settings = settingsRepository.settings.first()
            if (settings.serviceControlPin.isNotEmpty()) {
                if (enteredPin != settings.serviceControlPin) {
                    _uiState.update { it.copy(error = "Incorrect PIN") }
                    return@launch
                }
            }
            
            Log.d(TAG, "Stopping all services")
            val result = orinServiceRepository.stopServices()
            
            result.onSuccess { response ->
                Log.d(TAG, "Services stopped successfully: ${response.message}")
                // Wait a bit and refresh to show updated status
                delay(2000)
                refreshStatus()
            }.onFailure { error ->
                Log.e(TAG, "Failed to stop services: ${error.message}")
            }
        }
    }

    /**
     * Toggle auto-refresh mode
     */
    fun toggleAutoRefresh() {
        val newState = !_uiState.value.autoRefreshEnabled
        _uiState.update { it.copy(autoRefreshEnabled = newState) }
        
        if (newState) {
            startAutoRefresh()
        }
    }

    /**
     * Start auto-refresh loop
     */
    private fun startAutoRefresh() {
        viewModelScope.launch {
            while (_uiState.value.autoRefreshEnabled) {
                refreshStatus()
                delay(AUTO_REFRESH_INTERVAL_MS)
            }
        }
    }

    /**
     * Clear error message
     */
    fun clearError() {
        orinServiceRepository.clearError()
    }

    /**
     * Check if all services are running
     */
    fun areAllServicesRunning(): Boolean {
        return _uiState.value.services.values.all { it.running }
    }

    /**
     * Get service by ID
     */
    fun getService(serviceId: String): OrinServiceStatus? {
        return _uiState.value.services[serviceId]
    }
}
