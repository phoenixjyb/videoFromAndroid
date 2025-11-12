package com.example.camviewer.ui.screens.media

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.camviewer.data.model.MediaFilter
import com.example.camviewer.data.model.MediaItem
import com.example.camviewer.data.repository.MediaRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import java.io.File
import javax.inject.Inject

/**
 * ViewModel for Media Gallery screen
 */
@HiltViewModel
class MediaViewModel @Inject constructor(
    private val mediaRepository: MediaRepository
) : ViewModel() {

    companion object {
        private const val TAG = "MediaViewModel"
    }

    // Media list from repository
    val mediaItems: StateFlow<List<MediaItem>> = mediaRepository.mediaItems
        .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(5000),
            initialValue = emptyList()
        )

    // Loading state
    val isLoading: StateFlow<Boolean> = mediaRepository.isLoading
        .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(5000),
            initialValue = false
        )

    // Error message
    val error: StateFlow<String?> = mediaRepository.error
        .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(5000),
            initialValue = null
        )

    // Download progress for all media items
    val downloadProgress: StateFlow<Map<String, Float>> = mediaRepository.downloadProgress
        .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(5000),
            initialValue = emptyMap()
        )

    // Selected filter
    private val _currentFilter = MutableStateFlow<MediaFilter?>(null)
    val currentFilter: StateFlow<MediaFilter?> = _currentFilter.asStateFlow()

    // Refresh trigger - increments to force UI recomposition
    private val _refreshTrigger = MutableStateFlow(0)
    val refreshTrigger: StateFlow<Int> = _refreshTrigger.asStateFlow()

    // UI State
    private val _uiState = MutableStateFlow<MediaUiState>(MediaUiState.Empty)
    val uiState: StateFlow<MediaUiState> = combine(
        mediaItems,
        isLoading,
        error
    ) { items, loading, error ->
        when {
            error != null -> MediaUiState.Error(error)
            loading && items.isEmpty() -> MediaUiState.Loading
            items.isEmpty() -> MediaUiState.Empty
            else -> MediaUiState.Success(items)
        }
    }.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(5000),
        initialValue = MediaUiState.Empty
    )

    init {
        // Load media on initialization
        loadMedia()
    }

    /**
     * Load media list from Orin
     */
    fun loadMedia(filter: MediaFilter? = null) {
        viewModelScope.launch {
            Log.d(TAG, "Loading media with filter: $filter")
            _currentFilter.value = filter
            mediaRepository.fetchMediaList(page = 0, pageSize = 50, filter = filter)
        }
    }

    /**
     * Refresh media list
     */
    fun refresh() {
        viewModelScope.launch {
            Log.d(TAG, "Refreshing media list")
            mediaRepository.refresh()
        }
    }

    /**
     * Download a media item
     */
    fun downloadMedia(mediaItem: MediaItem) {
        viewModelScope.launch {
            // Check if already downloaded
            if (isMediaDownloaded(mediaItem)) {
                Log.d(TAG, "Media already downloaded: ${mediaItem.filename}")
                return@launch
            }
            
            Log.d(TAG, "Starting download: ${mediaItem.filename}")
            mediaRepository.downloadMedia(mediaItem)
                .catch { error ->
                    Log.e(TAG, "Download failed: ${mediaItem.filename}", error)
                    // TODO: Show error message to user
                }
                .collect { progress ->
                    Log.v(TAG, "Download progress for ${mediaItem.filename}: ${(progress * 100).toInt()}%")
                    // When download completes, trigger UI refresh
                    if (progress >= 1.0f) {
                        Log.d(TAG, "Download completed, triggering UI refresh")
                        _refreshTrigger.value++
                    }
                }
        }
    }

    /**
     * Check if media is downloaded
     */
    fun isMediaDownloaded(mediaItem: MediaItem): Boolean {
        return mediaRepository.isDownloaded(mediaItem)
    }
    
    /**
     * Get downloaded media file
     */
    fun getMediaFile(mediaItem: MediaItem): File? {
        return mediaRepository.getLocalFile(mediaItem)
    }

    /**
     * Check if media is currently downloading
     */
    fun isMediaDownloading(mediaId: String): Boolean {
        return mediaRepository.isDownloading(mediaId)
    }

    /**
     * Get download progress for a media item
     */
    fun getDownloadProgress(mediaId: String): Float? {
        return mediaRepository.getDownloadProgress(mediaId)
    }

    /**
     * Delete downloaded media
     */
    fun deleteLocalMedia(mediaItem: MediaItem) {
        viewModelScope.launch {
            Log.d(TAG, "Deleting local media: ${mediaItem.filename}")
            mediaRepository.deleteLocalMedia(mediaItem)
                .onSuccess {
                    Log.d(TAG, "Successfully deleted: ${mediaItem.filename}")
                    // Trigger UI refresh so checkmark updates to download button
                    _refreshTrigger.value++
                }
                .onFailure { error ->
                    Log.e(TAG, "Failed to delete: ${mediaItem.filename}", error)
                }
        }
    }

    /**
     * Apply filter
     */
    fun applyFilter(filter: MediaFilter) {
        loadMedia(filter)
    }

    /**
     * Clear filter
     */
    fun clearFilter() {
        loadMedia(null)
    }

    /**
     * Download thumbnail for media item
     */
    fun downloadThumbnail(mediaItem: MediaItem) {
        viewModelScope.launch {
            mediaRepository.downloadThumbnail(mediaItem)
                .onFailure { error ->
                    Log.e(TAG, "Failed to download thumbnail for ${mediaItem.filename}", error)
                }
        }
    }

    /**
     * Clear error message
     */
    fun clearError() {
        viewModelScope.launch {
            // Error is managed by repository, just reload
            refresh()
        }
    }
}

/**
 * UI state for media screen
 */
sealed class MediaUiState {
    object Loading : MediaUiState()
    object Empty : MediaUiState()
    data class Success(val items: List<MediaItem>) : MediaUiState()
    data class Error(val message: String) : MediaUiState()
}
