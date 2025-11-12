package com.example.camviewer.data.model

import kotlinx.serialization.Serializable

/**
 * Represents a media item (video or image) stored on Orin
 */
@Serializable
data class MediaItem(
    val id: String,                    // Unique identifier (filename or hash)
    val filename: String,              // Original filename
    val type: MediaType,               // VIDEO or IMAGE
    val timestamp: Long,               // Unix timestamp (milliseconds)
    val size: Long,                    // File size in bytes
    val resolution: Resolution? = null, // Video/image resolution
    val duration: Int? = null,         // Duration in seconds (for videos)
    val codec: String? = null,         // Codec (h264, h265)
    val fps: Int? = null,              // Frame rate (for videos)
    val bitrate: Int? = null,          // Bitrate in kbps
    val thumbnailUrl: String? = null,  // URL to thumbnail
    val downloadUrl: String            // URL to download full media
)

@Serializable
enum class MediaType {
    VIDEO,
    IMAGE
}

/**
 * Response from /media/list endpoint
 */
@Serializable
data class MediaListResponse(
    val items: List<MediaItem>,
    val total: Int,
    val page: Int,
    val pageSize: Int
)

/**
 * Download progress state
 */
data class DownloadProgress(
    val mediaId: String,
    val filename: String,
    val bytesDownloaded: Long,
    val totalBytes: Long,
    val percentage: Float
) {
    val isComplete: Boolean
        get() = bytesDownloaded >= totalBytes
}

/**
 * Downloaded media item stored locally
 */
data class LocalMedia(
    val mediaItem: MediaItem,
    val localPath: String,           // Path to local file
    val downloadedAt: Long,          // Unix timestamp when downloaded
    val thumbnailPath: String? = null // Path to local thumbnail
)

/**
 * Media filter options
 */
data class MediaFilter(
    val type: MediaType? = null,     // Filter by type
    val fromTimestamp: Long? = null, // Filter by date range
    val toTimestamp: Long? = null,
    val sortBy: SortField = SortField.TIMESTAMP,
    val sortOrder: SortOrder = SortOrder.DESCENDING
)

enum class SortField {
    TIMESTAMP,
    FILENAME,
    SIZE,
    DURATION
}

enum class SortOrder {
    ASCENDING,
    DESCENDING
}
