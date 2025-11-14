package com.example.camviewer.data.repository

import android.util.Log
import com.example.camviewer.data.model.OrinServiceStatus
import com.example.camviewer.data.model.ServiceControlResponse
import io.ktor.client.*
import io.ktor.client.call.*
import io.ktor.client.engine.cio.*
import io.ktor.client.plugins.*
import io.ktor.client.plugins.contentnegotiation.*
import io.ktor.client.request.*
import io.ktor.http.*
import io.ktor.serialization.kotlinx.json.*
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.serialization.json.Json
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Repository for controlling Orin services remotely
 */
@Singleton
class OrinServiceRepository @Inject constructor(
    private val settingsRepository: SettingsRepository
) {
    companion object {
        private const val TAG = "OrinServiceRepo"
        private const val SERVICE_CONTROL_PORT = 8083
    }

    private val client = HttpClient(CIO) {
        install(ContentNegotiation) {
            json(Json {
                ignoreUnknownKeys = true
                isLenient = true
            })
        }
        install(HttpTimeout) {
            requestTimeoutMillis = 60000  // 60 seconds for start/stop operations
            connectTimeoutMillis = 15000  // 15 seconds to connect
            socketTimeoutMillis = 60000   // 60 seconds socket timeout
        }
    }

    private val _servicesStatus = MutableStateFlow<Map<String, OrinServiceStatus>>(emptyMap())
    val servicesStatus: Flow<Map<String, OrinServiceStatus>> = _servicesStatus.asStateFlow()

    private val _isLoading = MutableStateFlow(false)
    val isLoading: Flow<Boolean> = _isLoading.asStateFlow()

    private val _error = MutableStateFlow<String?>(null)
    val error: Flow<String?> = _error.asStateFlow()

    /**
     * Get base URL for service control API
     */
    private suspend fun getServiceControlUrl(): String {
        val settings = settingsRepository.settings.first()
        // Extract Orin IP from target or media URL
        val orinIp = settings.orinTargetUrl
            .removePrefix("http://")
            .removePrefix("https://")
            .substringBefore(":")
        
        return "http://$orinIp:$SERVICE_CONTROL_PORT"
    }

    /**
     * Fetch current service status
     */
    suspend fun fetchStatus(): Result<Map<String, OrinServiceStatus>> {
        return try {
            _isLoading.value = true
            _error.value = null

            val baseUrl = getServiceControlUrl()
            val url = "$baseUrl/api/services/status"
            
            Log.d(TAG, "Fetching service status from: $url")
            
            val response = client.get(url)
            
            if (response.status.isSuccess()) {
                val statusMap: Map<String, OrinServiceStatus> = response.body()
                _servicesStatus.value = statusMap
                Log.d(TAG, "Service status updated: ${statusMap.keys}")
                Result.success(statusMap)
            } else {
                val errorMsg = "HTTP ${response.status.value}: ${response.status.description}"
                _error.value = errorMsg
                Log.e(TAG, errorMsg)
                Result.failure(Exception(errorMsg))
            }
        } catch (e: Exception) {
            val errorMsg = "Failed to fetch status: ${e.message}"
            _error.value = errorMsg
            Log.e(TAG, errorMsg, e)
            Result.failure(e)
        } finally {
            _isLoading.value = false
        }
    }

    /**
     * Start all Orin services
     */
    suspend fun startServices(): Result<ServiceControlResponse> {
        return try {
            _isLoading.value = true
            _error.value = null

            val baseUrl = getServiceControlUrl()
            val url = "$baseUrl/api/services/start"
            val settings = settingsRepository.settings.first()
            
            Log.d(TAG, "Starting services at: $url")
            Log.d(TAG, "PIN configured: ${settings.serviceControlPin.isNotEmpty()}")
            
            val response = client.post(url) {
                // Include PIN in header if set
                if (settings.serviceControlPin.isNotEmpty()) {
                    header("X-Service-PIN", settings.serviceControlPin)
                    Log.d(TAG, "PIN header added")
                }
            }
            
            Log.d(TAG, "Response status: ${response.status.value} ${response.status.description}")
            
            if (response.status.isSuccess()) {
                val controlResponse: ServiceControlResponse = response.body()
                _servicesStatus.value = controlResponse.services
                Log.d(TAG, "Services started: ${controlResponse.message}")
                Result.success(controlResponse)
            } else {
                val errorMsg = "HTTP ${response.status.value}: ${response.status.description}"
                _error.value = errorMsg
                Log.e(TAG, errorMsg)
                Result.failure(Exception(errorMsg))
            }
        } catch (e: Exception) {
            val errorMsg = "Failed to start services: ${e.message}"
            _error.value = errorMsg
            Log.e(TAG, errorMsg, e)
            Log.e(TAG, "Exception type: ${e.javaClass.name}")
            Log.e(TAG, "Cause: ${e.cause?.message}")
            Result.failure(e)
        } finally {
            _isLoading.value = false
        }
    }

    /**
     * Stop all Orin services
     */
    suspend fun stopServices(): Result<ServiceControlResponse> {
        return try {
            _isLoading.value = true
            _error.value = null

            val baseUrl = getServiceControlUrl()
            val url = "$baseUrl/api/services/stop"
            val settings = settingsRepository.settings.first()
            
            Log.d(TAG, "Stopping services at: $url")
            Log.d(TAG, "PIN configured: ${settings.serviceControlPin.isNotEmpty()}")
            
            val response = client.post(url) {
                // Include PIN in header if set
                if (settings.serviceControlPin.isNotEmpty()) {
                    header("X-Service-PIN", settings.serviceControlPin)
                    Log.d(TAG, "PIN header added")
                }
            }
            
            Log.d(TAG, "Response status: ${response.status.value} ${response.status.description}")
            
            if (response.status.isSuccess()) {
                val controlResponse: ServiceControlResponse = response.body()
                _servicesStatus.value = controlResponse.services
                Log.d(TAG, "Services stopped: ${controlResponse.message}")
                Result.success(controlResponse)
            } else {
                val errorMsg = "HTTP ${response.status.value}: ${response.status.description}"
                _error.value = errorMsg
                Log.e(TAG, errorMsg)
                Result.failure(Exception(errorMsg))
            }
        } catch (e: Exception) {
            val errorMsg = "Failed to stop services: ${e.message}"
            _error.value = errorMsg
            Log.e(TAG, errorMsg, e)
            Log.e(TAG, "Exception type: ${e.javaClass.name}")
            Log.e(TAG, "Cause: ${e.cause?.message}")
            Result.failure(e)
        } finally {
            _isLoading.value = false
        }
    }

    /**
     * Clear error state
     */
    fun clearError() {
        _error.value = null
    }
}
