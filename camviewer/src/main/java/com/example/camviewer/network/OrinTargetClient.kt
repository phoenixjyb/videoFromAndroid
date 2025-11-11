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
 * Sends tap coordinates or ROI (Region of Interest) to the Orin device for camera control.
 */
@Singleton
class OrinTargetClient @Inject constructor(
    @KtorHttpClient private val httpClient: HttpClient
) {
    companion object {
        private const val TAG = "OrinTargetClient"
    }

    /**
     * Sends target coordinates to the Orin device (simple tap point).
     * The server will create a default 10% bounding box around the point.
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
        return sendTargetROI(baseUrl, x, y, null, null)
    }

    /**
     * Sends target Region of Interest (ROI) to the Orin device.
     * 
     * @param baseUrl The base URL of the Orin target API (e.g., "http://172.16.30.234:8080")
     * @param x Normalized x coordinate - center point or top-left of ROI (0.0 - 1.0)
     * @param y Normalized y coordinate - center point or top-left of ROI (0.0 - 1.0)
     * @param width Normalized width of ROI (0.0 - 1.0), null for simple tap point
     * @param height Normalized height of ROI (0.0 - 1.0), null for simple tap point
     * @return Result indicating success or failure with error message
     */
    suspend fun sendTargetROI(
        baseUrl: String,
        x: Float,
        y: Float,
        width: Float?,
        height: Float?
    ): Result<Unit> {
        return try {
            // Validate coordinates
            if (x !in 0.0f..1.0f || y !in 0.0f..1.0f) {
                Log.w(TAG, "Invalid coordinates: x=$x, y=$y (must be 0.0-1.0)")
                return Result.failure(IllegalArgumentException("Coordinates must be between 0.0 and 1.0"))
            }

            // Validate width/height if provided
            if (width != null && width !in 0.0f..1.0f) {
                Log.w(TAG, "Invalid width: $width (must be 0.0-1.0)")
                return Result.failure(IllegalArgumentException("Width must be between 0.0 and 1.0"))
            }
            if (height != null && height !in 0.0f..1.0f) {
                Log.w(TAG, "Invalid height: $height (must be 0.0-1.0)")
                return Result.failure(IllegalArgumentException("Height must be between 0.0 and 1.0"))
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

            // Log appropriate message
            if (width != null && height != null) {
                Log.d(TAG, "Sending target ROI: x=$x, y=$y, w=$width, h=$height to $targetUrl")
            } else {
                Log.d(TAG, "Sending target point: x=$x, y=$y to $targetUrl")
            }

            // Send POST request with appropriate body
            val response: HttpResponse = httpClient.post(targetUrl) {
                contentType(ContentType.Application.Json)
                setBody(
                    if (width != null && height != null) {
                        TargetROIRequest(x, y, width, height)
                    } else {
                        TargetPointRequest(x, y)
                    }
                )
            }

            if (response.status.isSuccess()) {
                Log.i(TAG, "Target sent successfully: ${response.status}")
                Result.success(Unit)
            } else {
                val error = "Failed to send target: ${response.status}"
                Log.e(TAG, error)
                Result.failure(Exception(error))
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error sending target", e)
            Result.failure(e)
        }
    }

    /**
     * Request body for sending simple target point (tap).
     */
    @Serializable
    private data class TargetPointRequest(
        val x: Float,
        val y: Float
    )

    /**
     * Request body for sending target ROI (bounding box).
     */
    @Serializable
    private data class TargetROIRequest(
        val x: Float,
        val y: Float,
        val width: Float,
        val height: Float
    )
}
