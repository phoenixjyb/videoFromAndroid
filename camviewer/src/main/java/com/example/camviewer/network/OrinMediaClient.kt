package com.example.camviewer.network

import android.util.Log
import com.example.camviewer.data.model.MediaFilter
import com.example.camviewer.data.model.MediaItem
import com.example.camviewer.data.model.MediaListResponse
import com.example.camviewer.di.KtorHttpClient
import io.ktor.client.*
import io.ktor.client.call.*
import io.ktor.client.plugins.*
import io.ktor.client.request.*
import io.ktor.client.statement.*
import io.ktor.http.*
import io.ktor.utils.io.*
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.withContext
import java.io.File
import javax.inject.Inject
import javax.inject.Singleton

/**
 * HTTP client for Orin Media API
 * Handles listing media files and downloading them from Orin device
 */
@Singleton
class OrinMediaClient @Inject constructor(
    @KtorHttpClient private val httpClient: HttpClient
) {
    companion object {
        private const val TAG = "OrinMediaClient"
    }

    /**
     * List all media files from Orin
     * 
     * @param baseUrl Base URL of Orin media API (e.g., "http://172.16.30.234:8081")
     * @param page Page number (0-indexed)
     * @param pageSize Number of items per page
     * @param filter Optional filter criteria
     * @return Result containing list of media items
     */
    suspend fun listMedia(
        baseUrl: String,
        page: Int = 0,
        pageSize: Int = 50,
        filter: MediaFilter? = null
    ): Result<MediaListResponse> = withContext(Dispatchers.IO) {
        runCatching {
            Log.d(TAG, "Fetching media list from $baseUrl (page=$page, pageSize=$pageSize)")
            
            val response = httpClient.get("$baseUrl/media/list") {
                parameter("page", page)
                parameter("pageSize", pageSize)
                
                filter?.let {
                    it.type?.let { type -> parameter("type", type.name.lowercase()) }
                    it.fromTimestamp?.let { ts -> parameter("from", ts) }
                    it.toTimestamp?.let { ts -> parameter("to", ts) }
                    parameter("sortBy", it.sortBy.name.lowercase())
                    parameter("sortOrder", it.sortOrder.name.lowercase())
                }
                
                timeout {
                    requestTimeoutMillis = 10000 // 10 seconds
                }
            }
            
            if (response.status.isSuccess()) {
                val mediaList: MediaListResponse = response.body()
                Log.d(TAG, "Successfully fetched ${mediaList.items.size} media items")
                mediaList
            } else {
                Log.e(TAG, "Failed to fetch media list: ${response.status}")
                throw Exception("HTTP ${response.status.value}: ${response.status.description}")
            }
        }.onFailure { error ->
            Log.e(TAG, "Error fetching media list", error)
        }
    }

    /**
     * Get details for a specific media item
     * 
     * @param baseUrl Base URL of Orin media API
     * @param mediaId Media item ID
     * @return Result containing media item details
     */
    suspend fun getMediaDetails(
        baseUrl: String,
        mediaId: String
    ): Result<MediaItem> = withContext(Dispatchers.IO) {
        runCatching {
            Log.d(TAG, "Fetching media details for $mediaId from $baseUrl")
            
            val response = httpClient.get("$baseUrl/media/$mediaId") {
                timeout {
                    requestTimeoutMillis = 5000 // 5 seconds
                }
            }
            
            if (response.status.isSuccess()) {
                val mediaItem: MediaItem = response.body()
                Log.d(TAG, "Successfully fetched media details: ${mediaItem.filename}")
                mediaItem
            } else {
                Log.e(TAG, "Failed to fetch media details: ${response.status}")
                throw Exception("HTTP ${response.status.value}: ${response.status.description}")
            }
        }.onFailure { error ->
            Log.e(TAG, "Error fetching media details for $mediaId", error)
        }
    }

    /**
     * Download a media file from Orin with progress tracking
     * 
     * @param downloadUrl Full URL to download the media file
     * @param destination Local file where media will be saved
     * @return Flow emitting download progress (0.0 to 1.0)
     */
    fun downloadMedia(
        downloadUrl: String,
        destination: File
    ): Flow<Float> = flow {
        Log.d(TAG, "Starting download: $downloadUrl -> ${destination.absolutePath}")
        
        try {
            var totalBytesDownloaded = 0L
            
            httpClient.prepareGet(downloadUrl) {
                timeout {
                    requestTimeoutMillis = HttpTimeout.INFINITE_TIMEOUT_MS
                }
            }.execute { response ->
                if (!response.status.isSuccess()) {
                    throw Exception("HTTP ${response.status.value}: ${response.status.description}")
                }
                
                val contentLength = response.contentLength() ?: -1L
                Log.d(TAG, "Download content length: $contentLength bytes")
                
                val channel: ByteReadChannel = response.bodyAsChannel()
                
                destination.parentFile?.mkdirs()
                destination.outputStream().use { output ->
                    val buffer = ByteArray(8192) // 8KB buffer
                    
                    while (!channel.isClosedForRead) {
                        val bytesRead = channel.readAvailable(buffer, 0, buffer.size)
                        if (bytesRead == -1) break
                        
                        output.write(buffer, 0, bytesRead)
                        totalBytesDownloaded += bytesRead
                        
                        // Emit progress
                        if (contentLength > 0) {
                            val progress = totalBytesDownloaded.toFloat() / contentLength.toFloat()
                            emit(progress)
                            Log.v(TAG, "Download progress: ${(progress * 100).toInt()}%")
                        } else {
                            // Unknown size, emit bytes downloaded
                            emit(totalBytesDownloaded.toFloat())
                        }
                    }
                    
                    output.flush()
                }
                
                Log.d(TAG, "Download complete: ${destination.absolutePath} ($totalBytesDownloaded bytes)")
                emit(1.0f) // Complete
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error downloading media from $downloadUrl", e)
            destination.delete() // Clean up partial download
            throw e
        }
    }

    /**
     * Download thumbnail for a media item
     * 
     * @param thumbnailUrl Full URL to thumbnail
     * @param destination Local file where thumbnail will be saved
     * @return Result indicating success or failure
     */
    suspend fun downloadThumbnail(
        thumbnailUrl: String,
        destination: File
    ): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            Log.d(TAG, "Downloading thumbnail: $thumbnailUrl -> ${destination.absolutePath}")
            
            val response = httpClient.get(thumbnailUrl) {
                timeout {
                    requestTimeoutMillis = 10000 // 10 seconds for thumbnails
                }
            }
            
            if (response.status.isSuccess()) {
                destination.parentFile?.mkdirs()
                destination.writeBytes(response.body<ByteArray>())
                Log.d(TAG, "Thumbnail downloaded: ${destination.absolutePath}")
            } else {
                Log.e(TAG, "Failed to download thumbnail: ${response.status}")
                throw Exception("HTTP ${response.status.value}: ${response.status.description}")
            }
        }.onFailure { error ->
            Log.e(TAG, "Error downloading thumbnail from $thumbnailUrl", error)
            destination.delete() // Clean up partial download
        }.map { Unit } // Ensure Result<Unit>
    }

    /**
     * Delete a media file on Orin (if API supports it)
     * 
     * @param baseUrl Base URL of Orin media API
     * @param mediaId Media item ID to delete
     * @return Result indicating success or failure
     */
    suspend fun deleteMedia(
        baseUrl: String,
        mediaId: String
    ): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            Log.d(TAG, "Deleting media: $mediaId from $baseUrl")
            
            val response = httpClient.delete("$baseUrl/media/$mediaId") {
                timeout {
                    requestTimeoutMillis = 5000
                }
            }
            
            if (response.status.isSuccess()) {
                Log.d(TAG, "Successfully deleted media: $mediaId")
            } else {
                Log.e(TAG, "Failed to delete media: ${response.status}")
                throw Exception("HTTP ${response.status.value}: ${response.status.description}")
            }
        }.onFailure { error ->
            Log.e(TAG, "Error deleting media $mediaId", error)
        }.map { Unit } // Ensure Result<Unit>
    }
}
