package com.example.camviewer.network

import android.util.Log
import io.ktor.client.HttpClient
import io.ktor.client.request.post
import io.ktor.client.request.setBody
import io.ktor.client.statement.HttpResponse
import io.ktor.http.ContentType
import io.ktor.http.contentType
import io.ktor.http.isSuccess
import kotlinx.serialization.Serializable
import javax.inject.Inject
import javax.inject.Singleton
import com.example.camviewer.di.KtorHttpClient

/**
 * Client for communicating with the Orin target API.
 * Sends tap coordinates to the Orin device for camera control.
 */
@Singleton
class OrinTargetClient @Inject constructor(
    @KtorHttpClient private val httpClient: HttpClient
) {
    companion object {
        private const val TAG = "OrinTargetClient"
    }

    /**
     * Sends target coordinates to the Orin device.
     * 
     * @param baseUrl The base URL of the Orin target API (e.g., "http://172.16.30.234:8080")
     * @param x Normalized x coordinate (0.0 - 1.0)
     * @param y Normalized y coordinate (0.0 - 1.0)
     * @return Result indicating success or failure with error message
     */
    suspend fun sendTargetCoordinates(
        baseUrl: String,
        x: Float,
        y: Float
    ): Result<Unit> {
        return try {
            // Validate coordinates
            if (x !in 0.0f..1.0f || y !in 0.0f..1.0f) {
                Log.w(TAG, "Invalid coordinates: x=$x, y=$y (must be 0.0-1.0)")
                return Result.failure(IllegalArgumentException("Coordinates must be between 0.0 and 1.0"))
            }

            // Validate base URL
            val url = baseUrl.trim()
            if (url.isBlank()) {
                Log.w(TAG, "Empty Orin target URL")
                return Result.failure(IllegalArgumentException("Orin target URL is not configured"))
            }

            // Construct full URL
            val targetUrl = if (url.endsWith("/")) {
                "${url}target"
            } else {
                "$url/target"
            }

            Log.d(TAG, "Sending target coordinates: x=$x, y=$y to $targetUrl")

            // Send POST request
            val response: HttpResponse = httpClient.post(targetUrl) {
                contentType(ContentType.Application.Json)
                setBody(TargetRequest(x, y))
            }

            if (response.status.isSuccess()) {
                Log.i(TAG, "Target coordinates sent successfully: ${response.status}")
                Result.success(Unit)
            } else {
                val error = "Failed to send target coordinates: ${response.status}"
                Log.e(TAG, error)
                Result.failure(Exception(error))
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error sending target coordinates", e)
            Result.failure(e)
        }
    }

    /**
     * Request body for sending target coordinates.
     */
    @Serializable
    private data class TargetRequest(
        val x: Float,
        val y: Float
    )
}
