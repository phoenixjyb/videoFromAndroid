package com.example.camviewer.data.repository

import android.content.Context
import android.util.Log
import com.example.camviewer.data.model.*
import com.example.camviewer.network.OrinMediaClient
import dagger.hilt.android.qualifiers.ApplicationContext
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.withContext
import java.io.File
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Repository for managing media files from Orin
 * Handles fetching, caching, and downloading media
 */
@Singleton
class MediaRepository @Inject constructor(
    @ApplicationContext private val context: Context,
    private val orinMediaClient: OrinMediaClient,
    private val settingsRepository: SettingsRepository
) {
    companion object {
        private const val TAG = "MediaRepository"
        private const val MEDIA_CACHE_DIR = "media_cache"
        private const val THUMBNAIL_CACHE_DIR = "thumbnails"
    }

    private val _mediaItems = MutableStateFlow<List<MediaItem>>(emptyList())
    val mediaItems: StateFlow<List<MediaItem>> = _mediaItems.asStateFlow()

    private val _isLoading = MutableStateFlow(false)
    val isLoading: StateFlow<Boolean> = _isLoading.asStateFlow()

    private val _error = MutableStateFlow<String?>(null)
    val error: StateFlow<String?> = _error.asStateFlow()

    private val _downloadProgress = MutableStateFlow<Map<String, Float>>(emptyMap())
    val downloadProgress: StateFlow<Map<String, Float>> = _downloadProgress.asStateFlow()

    /**
     * Get media cache directory
     */
    private fun getMediaCacheDir(): File {
        val cacheDir = File(context.cacheDir, MEDIA_CACHE_DIR)
        if (!cacheDir.exists()) {
            cacheDir.mkdirs()
        }
        return cacheDir
    }

    /**
     * Get thumbnail cache directory
     */
    private fun getThumbnailCacheDir(): File {
        val cacheDir = File(context.cacheDir, THUMBNAIL_CACHE_DIR)
        if (!cacheDir.exists()) {
            cacheDir.mkdirs()
        }
        return cacheDir
    }

    /**
     * Get downloads directory (permanent storage)
     */
    private fun getDownloadsDir(): File {
        val downloadsDir = File(context.getExternalFilesDir(null), "Downloads")
        if (!downloadsDir.exists()) {
            downloadsDir.mkdirs()
        }
        return downloadsDir
    }

    /**
     * Fetch media list from Orin
     */
    suspend fun fetchMediaList(
        page: Int = 0,
        pageSize: Int = 50,
        filter: MediaFilter? = null
    ): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            _isLoading.value = true
            _error.value = null

            val settings = settingsRepository.settings.first()
            val baseUrl = settings.orinMediaUrl

            Log.d(TAG, "Fetching media list from $baseUrl")

            val result = orinMediaClient.listMedia(baseUrl, page, pageSize, filter)
            
            if (result.isSuccess) {
                val response = result.getOrThrow()
                _mediaItems.value = if (page == 0) {
                    // First page, replace all items
                    response.items
                } else {
                    // Append to existing items (pagination)
                    _mediaItems.value + response.items
                }
                Log.d(TAG, "Media list updated: ${_mediaItems.value.size} items")
            } else {
                val error = result.exceptionOrNull()
                _error.value = "Failed to load media: ${error?.message}"
                Log.e(TAG, "Failed to fetch media list", error)
                throw error ?: Exception("Unknown error")
            }
            Unit // Explicitly return Unit
        }.also {
            _isLoading.value = false
        }
    }

    /**
     * Get cached thumbnail file for a media item
     */
    fun getCachedThumbnail(mediaItem: MediaItem): File? {
        val thumbnailFile = File(getThumbnailCacheDir(), "${mediaItem.id}_thumb.jpg")
        return if (thumbnailFile.exists()) thumbnailFile else null
    }

    /**
     * Download thumbnail for a media item
     */
    suspend fun downloadThumbnail(mediaItem: MediaItem): Result<File> = withContext(Dispatchers.IO) {
        runCatching {
            // Check if already cached
            getCachedThumbnail(mediaItem)?.let { return@runCatching it }

            // Download thumbnail
            val thumbnailUrl = mediaItem.thumbnailUrl
                ?: throw IllegalArgumentException("Media item has no thumbnail URL")

            val destination = File(getThumbnailCacheDir(), "${mediaItem.id}_thumb.jpg")

            orinMediaClient.downloadThumbnail(thumbnailUrl, destination)
                .onFailure { error ->
                    Log.e(TAG, "Failed to download thumbnail for ${mediaItem.filename}", error)
                    throw error
                }

            destination
        }
    }

    /**
     * Download media file to downloads directory with progress tracking
     */
    fun downloadMedia(mediaItem: MediaItem): Flow<Float> = flow {
        try {
            val destination = File(getDownloadsDir(), mediaItem.filename)

            // Check if already downloaded with correct size
            if (destination.exists() && destination.length() == mediaItem.size) {
                Log.d(TAG, "Media already downloaded: ${mediaItem.filename}")
                emit(1.0f)
                return@flow
            }

            // If file exists but size doesn't match, delete it
            if (destination.exists()) {
                Log.w(TAG, "Existing file size mismatch for ${mediaItem.filename}, deleting (expected: ${mediaItem.size}, actual: ${destination.length()})")
                destination.delete()
            }

            Log.d(TAG, "Starting download: ${mediaItem.filename}")

            orinMediaClient.downloadMedia(mediaItem.downloadUrl, destination)
                .collect { progress ->
                    // Update progress map
                    _downloadProgress.value = _downloadProgress.value.toMutableMap().apply {
                        put(mediaItem.id, progress)
                    }
                    emit(progress)
                }

            // Remove from progress map when complete
            _downloadProgress.value = _downloadProgress.value.toMutableMap().apply {
                remove(mediaItem.id)
            }

            Log.d(TAG, "Download complete: ${mediaItem.filename}, size: ${destination.length()} bytes")
        } catch (e: Exception) {
            Log.e(TAG, "Download failed for ${mediaItem.filename}", e)
            _downloadProgress.value = _downloadProgress.value.toMutableMap().apply {
                remove(mediaItem.id)
            }
            throw e
        }
    }.flowOn(Dispatchers.IO)

    /**
     * Get download progress for a specific media item
     */
    fun getDownloadProgress(mediaId: String): Float? {
        return _downloadProgress.value[mediaId]
    }

    /**
     * Check if media is currently downloading
     */
    fun isDownloading(mediaId: String): Boolean {
        return _downloadProgress.value.containsKey(mediaId)
    }

    /**
     * Check if media is already downloaded
     */
    fun isDownloaded(mediaItem: MediaItem): Boolean {
        val file = File(getDownloadsDir(), mediaItem.filename)
        return file.exists() && file.length() == mediaItem.size
    }

    /**
     * Get local file for a downloaded media item
     */
    fun getLocalFile(mediaItem: MediaItem): File? {
        val file = File(getDownloadsDir(), mediaItem.filename)
        return if (file.exists()) file else null
    }

    /**
     * Delete downloaded media file
     */
    suspend fun deleteLocalMedia(mediaItem: MediaItem): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            val file = File(getDownloadsDir(), mediaItem.filename)
            if (file.exists()) {
                val deleted = file.delete()
                if (deleted) {
                    Log.d(TAG, "Deleted local media: ${mediaItem.filename}")
                } else {
                    Log.e(TAG, "Failed to delete local media: ${mediaItem.filename}")
                }
            } else {
                Log.d(TAG, "File doesn't exist, nothing to delete: ${mediaItem.filename}")
            }

            // Also delete thumbnail
            getCachedThumbnail(mediaItem)?.let { thumbFile ->
                if (thumbFile.exists()) {
                    thumbFile.delete()
                    Log.d(TAG, "Deleted thumbnail: ${thumbFile.name}")
                }
            }
            
            // Remove from download progress if present
            _downloadProgress.value = _downloadProgress.value.toMutableMap().apply {
                remove(mediaItem.id)
            }
            
            Unit // Explicitly return Unit
        }
    }

    /**
     * Clear thumbnail cache
     */
    suspend fun clearThumbnailCache(): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            val cacheDir = getThumbnailCacheDir()
            cacheDir.listFiles()?.forEach { it.delete() }
            Log.d(TAG, "Cleared thumbnail cache")
            Unit // Explicitly return Unit
        }
    }

    /**
     * Get cache size
     */
    suspend fun getCacheSize(): Long = withContext(Dispatchers.IO) {
        val mediaCache = getMediaCacheDir().walkTopDown().filter { it.isFile }.map { it.length() }.sum()
        val thumbnailCache = getThumbnailCacheDir().walkTopDown().filter { it.isFile }.map { it.length() }.sum()
        mediaCache + thumbnailCache
    }

    /**
     * Get downloads size
     */
    suspend fun getDownloadsSize(): Long = withContext(Dispatchers.IO) {
        getDownloadsDir().walkTopDown().filter { it.isFile }.map { it.length() }.sum()
    }

    /**
     * Refresh media list (convenience method)
     */
    suspend fun refresh() {
        fetchMediaList(page = 0, pageSize = 50)
    }
}
