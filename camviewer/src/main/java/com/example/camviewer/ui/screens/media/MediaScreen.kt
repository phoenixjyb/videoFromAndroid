package com.example.camviewer.ui.screens.media

import android.net.Uri
import android.widget.Toast
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.foundation.lazy.grid.items
import androidx.compose.material.ripple.rememberRipple
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import androidx.hilt.navigation.compose.hiltViewModel
import coil.compose.AsyncImage
import com.example.camviewer.data.model.MediaItem
import com.example.camviewer.data.model.MediaType
import androidx.media3.common.MediaItem as ExoMediaItem
import androidx.media3.common.PlaybackException
import androidx.media3.common.Player
import androidx.media3.common.util.UnstableApi
import androidx.media3.exoplayer.ExoPlayer
import androidx.media3.ui.PlayerView
import java.io.File
import java.text.SimpleDateFormat
import java.util.*

enum class MediaTab {
    LOCAL_RECORDINGS,
    SYNCED_VIDEOS
}

@Composable
fun MediaScreen(
    viewModel: MediaViewModel = hiltViewModel(),
    onNavigateBack: () -> Unit = {}
) {
    val uiState by viewModel.uiState.collectAsState()
    val downloadProgress by viewModel.downloadProgress.collectAsState()
    val refreshTrigger by viewModel.refreshTrigger.collectAsState() // Trigger UI updates
    val localRecordings by viewModel.localRecordings.collectAsState()
    var selectedMedia by remember { mutableStateOf<MediaItem?>(null) }
    var mediaToDelete by remember { mutableStateOf<MediaItem?>(null) }
    var selectedTab by remember { mutableStateOf(MediaTab.LOCAL_RECORDINGS) }
    var selectedLocalRecording by remember { mutableStateOf<File?>(null) }
    val context = LocalContext.current
    
    Scaffold(
        topBar = {
            MediaTopBar(
                selectedTab = selectedTab,
                onTabSelected = { selectedTab = it },
                onNavigateBack = onNavigateBack,
                onRefresh = { 
                    when (selectedTab) {
                        MediaTab.LOCAL_RECORDINGS -> viewModel.refreshLocalRecordings()
                        MediaTab.SYNCED_VIDEOS -> viewModel.refresh()
                    }
                }
            )
        }
    ) { paddingValues ->
        Box(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues)
        ) {
            when (selectedTab) {
                MediaTab.LOCAL_RECORDINGS -> {
                    // Show local recordings from /sdcard/Movies/recomoVideosRawStream/
                    LocalRecordingsContent(
                        recordings = localRecordings,
                        onMediaClick = { file -> selectedLocalRecording = file },
                        onDeleteClick = { viewModel.deleteLocalRecording(it) },
                        onRefresh = { viewModel.refreshLocalRecordings() }
                    )
                }
                MediaTab.SYNCED_VIDEOS -> {
                    // Show synced videos from Orin
                    when (val state = uiState) {
                        is MediaUiState.Loading -> {
                            LoadingContent()
                        }
                        is MediaUiState.Empty -> {
                            EmptyContent(onRefresh = { viewModel.refresh() })
                        }
                        is MediaUiState.Success -> {
                            MediaGrid(
                                items = state.items,
                                downloadProgress = downloadProgress,
                                onMediaClick = { mediaItem ->
                                    if (viewModel.isMediaDownloaded(mediaItem)) {
                                        android.util.Log.e("MediaScreen", "onMediaClick called for downloaded media: ${mediaItem.filename}")
                                        selectedMedia = mediaItem
                                    } else {
                                        android.util.Log.w("MediaScreen", "Attempted playback without download: ${mediaItem.filename}")
                                        Toast.makeText(
                                            context,
                                            "Please download the video before playback",
                                            Toast.LENGTH_SHORT
                                        ).show()
                                    }
                                },
                                onDownloadClick = { mediaItem ->
                                    viewModel.downloadMedia(mediaItem)
                                },
                                onDeleteClick = { mediaItem ->
                                    mediaToDelete = mediaItem
                                },
                                isDownloaded = { mediaItem ->
                                    viewModel.isMediaDownloaded(mediaItem)
                                }
                            )
                        }
                        is MediaUiState.Error -> {
                            ErrorContent(
                                message = state.message,
                                onRetry = { viewModel.refresh() }
                            )
                        }
                    }
                }
            }
        }
    }
    
    // Video player dialog
    selectedMedia?.let { media ->
        VideoPlayerDialog(
            mediaItem = media,
            onDismiss = { selectedMedia = null },
            getVideoFile = { viewModel.getMediaFile(it) }
        )
    }

    selectedLocalRecording?.let { localFile ->
        LocalVideoPlayerDialog(
            file = localFile,
            onDismiss = { selectedLocalRecording = null }
        )
    }
    
    // Delete confirmation dialog
    mediaToDelete?.let { media ->
        AlertDialog(
            onDismissRequest = { mediaToDelete = null },
            title = { Text("Delete Video") },
            text = { Text("Are you sure you want to delete '${media.filename}'? This will remove the downloaded file from your device.") },
            confirmButton = {
                TextButton(
                    onClick = {
                        android.util.Log.d("MediaScreen", "Deleting local media: ${media.filename}")
                        viewModel.deleteLocalMedia(media)
                        mediaToDelete = null
                    }
                ) {
                    Text("Delete", color = MaterialTheme.colorScheme.error)
                }
            },
            dismissButton = {
                TextButton(onClick = { mediaToDelete = null }) {
                    Text("Cancel")
                }
            }
        )
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MediaTopBar(
    selectedTab: MediaTab,
    onTabSelected: (MediaTab) -> Unit,
    onNavigateBack: () -> Unit,
    onRefresh: () -> Unit
) {
    Column {
        TopAppBar(
            title = { Text("Media Gallery") },
            navigationIcon = {
                IconButton(onClick = onNavigateBack) {
                    Icon(Icons.Default.ArrowBack, contentDescription = "Back")
                }
            },
            actions = {
                IconButton(onClick = onRefresh) {
                    Icon(Icons.Default.Refresh, contentDescription = "Refresh")
                }
            }
        )
        
        // Tab Row
        TabRow(
            selectedTabIndex = selectedTab.ordinal,
            modifier = Modifier.fillMaxWidth()
        ) {
            Tab(
                selected = selectedTab == MediaTab.LOCAL_RECORDINGS,
                onClick = { onTabSelected(MediaTab.LOCAL_RECORDINGS) },
                text = { Text("Local Recordings") },
                icon = { Icon(Icons.Default.Phone, contentDescription = "Local") }
            )
            Tab(
                selected = selectedTab == MediaTab.SYNCED_VIDEOS,
                onClick = { onTabSelected(MediaTab.SYNCED_VIDEOS) },
                text = { Text("Synced from Orin") },
                icon = { Icon(Icons.Default.CloudDownload, contentDescription = "Synced") }
            )
        }
    }
}

@Composable
fun MediaGrid(
    items: List<MediaItem>,
    downloadProgress: Map<String, Float>,
    onMediaClick: (MediaItem) -> Unit,
    onDownloadClick: (MediaItem) -> Unit,
    onDeleteClick: (MediaItem) -> Unit,
    isDownloaded: (MediaItem) -> Boolean
) {
    LazyVerticalGrid(
        columns = GridCells.Adaptive(minSize = 150.dp),
        contentPadding = PaddingValues(8.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        items(items, key = { it.id }) { mediaItem ->
            MediaCard(
                mediaItem = mediaItem,
                downloadProgress = downloadProgress[mediaItem.id],
                isDownloaded = isDownloaded(mediaItem),
                onClick = { onMediaClick(mediaItem) },
                onDownloadClick = { onDownloadClick(mediaItem) },
                onDeleteClick = { onDeleteClick(mediaItem) }
            )
        }
    }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun MediaCard(
    mediaItem: MediaItem,
    downloadProgress: Float?,
    isDownloaded: Boolean,
    onClick: () -> Unit,
    onDownloadClick: () -> Unit,
    onDeleteClick: () -> Unit
) {
    val context = LocalContext.current
    val interactionSource = remember { MutableInteractionSource() }
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .aspectRatio(1f)
            .combinedClickable(
                interactionSource = interactionSource,
                indication = rememberRipple(),
                onClick = {
                    android.util.Log.d(
                        "MediaScreen",
                        "Card tapped: ${mediaItem.filename}, downloaded: $isDownloaded"
                    )
                    onClick()
                },
                onLongClick = {
                    android.util.Log.d(
                        "MediaScreen",
                        "Long press on: ${mediaItem.filename}, downloaded: $isDownloaded"
                    )
                    if (isDownloaded) {
                        onDeleteClick()
                    } else {
                        Toast.makeText(
                            context,
                            "Download the video before deleting",
                            Toast.LENGTH_SHORT
                        ).show()
                    }
                }
            ),
        elevation = CardDefaults.cardElevation(defaultElevation = 2.dp)
    ) {
        Box(modifier = Modifier.fillMaxSize()) {
            // Thumbnail
            if (mediaItem.thumbnailUrl != null) {
                android.util.Log.d("MediaScreen", "Loading thumbnail for ${mediaItem.filename}: ${mediaItem.thumbnailUrl}")
                AsyncImage(
                    model = mediaItem.thumbnailUrl,
                    contentDescription = mediaItem.filename,
                    modifier = Modifier.fillMaxSize(),
                    contentScale = ContentScale.Crop,
                    onError = { error ->
                        android.util.Log.e("MediaScreen", "Failed to load thumbnail for ${mediaItem.filename}: ${error.result.throwable.message}")
                    },
                    onSuccess = {
                        android.util.Log.d("MediaScreen", "Successfully loaded thumbnail for ${mediaItem.filename}")
                    }
                )
            } else {
                android.util.Log.w("MediaScreen", "No thumbnail URL for ${mediaItem.filename}")
                // Placeholder for missing thumbnail
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(MaterialTheme.colorScheme.surfaceVariant),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        imageVector = if (mediaItem.type == MediaType.VIDEO) 
                            Icons.Default.VideoFile 
                        else 
                            Icons.Default.Image,
                        contentDescription = null,
                        modifier = Modifier.size(48.dp),
                        tint = MaterialTheme.colorScheme.onSurfaceVariant
                    )
                }
            }
            
            // Media type indicator
            Surface(
                modifier = Modifier
                    .align(Alignment.TopStart)
                    .padding(4.dp),
                shape = RoundedCornerShape(4.dp),
                color = Color.Black.copy(alpha = 0.6f)
            ) {
                Row(
                    modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(2.dp)
                ) {
                    Icon(
                        imageVector = if (mediaItem.type == MediaType.VIDEO) 
                            Icons.Default.PlayArrow 
                        else 
                            Icons.Default.Image,
                        contentDescription = null,
                        modifier = Modifier.size(12.dp),
                        tint = Color.White
                    )
                    if (mediaItem.duration != null) {
                        Text(
                            text = formatDuration(mediaItem.duration),
                            style = MaterialTheme.typography.labelSmall,
                            color = Color.White
                        )
                    }
                }
            }
            
            // Download indicator
            if (downloadProgress != null) {
                // Downloading
                Box(
                    modifier = Modifier
                        .align(Alignment.Center)
                        .size(60.dp)
                        .background(
                            color = Color.Black.copy(alpha = 0.7f),
                            shape = RoundedCornerShape(30.dp)
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    CircularProgressIndicator(
                        progress = { downloadProgress },
                        modifier = Modifier.size(40.dp),
                        color = Color.White,
                        trackColor = Color.Gray
                    )
                    Text(
                        text = "${(downloadProgress * 100).toInt()}%",
                        style = MaterialTheme.typography.labelSmall,
                        color = Color.White
                    )
                }
            } else if (!isDownloaded) {
                // Not downloaded, show download button
                IconButton(
                    onClick = {
                        android.util.Log.e("MediaScreen", "========================================")
                        android.util.Log.e("MediaScreen", "DOWNLOAD BUTTON CLICKED!!!")
                        android.util.Log.e("MediaScreen", "File: ${mediaItem.filename}")
                        android.util.Log.e("MediaScreen", "ID: ${mediaItem.id}")
                        android.util.Log.e("MediaScreen", "Calling onDownloadClick()...")
                        onDownloadClick()
                        android.util.Log.e("MediaScreen", "onDownloadClick() returned")
                        android.util.Log.e("MediaScreen", "========================================")
                    },
                    modifier = Modifier
                        .align(Alignment.BottomEnd)
                        .size(64.dp)  // Even larger clickable area
                        .background(
                            color = Color.Black.copy(alpha = 0.6f),
                            shape = RoundedCornerShape(32.dp)
                        )
                ) {
                    Icon(
                        imageVector = Icons.Default.Download,
                        contentDescription = "Download",
                        tint = Color.White,
                        modifier = Modifier.size(32.dp)
                    )
                }
            } else {
                // Downloaded, show checkmark
                Icon(
                    imageVector = Icons.Default.CheckCircle,
                    contentDescription = "Downloaded",
                    modifier = Modifier
                        .align(Alignment.BottomEnd)
                        .padding(4.dp)
                        .size(24.dp),
                    tint = Color.Green
                )
            }
            
            // Bottom info bar
            Surface(
                modifier = Modifier
                    .align(Alignment.BottomStart)
                    .fillMaxWidth()
                    .height(40.dp),
                color = Color.Black.copy(alpha = 0.6f)
            ) {
                Column(
                    modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp),
                    verticalArrangement = Arrangement.Center
                ) {
                    Text(
                        text = mediaItem.filename,
                        style = MaterialTheme.typography.labelSmall,
                        color = Color.White,
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis
                    )
                    Row(
                        horizontalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        Text(
                            text = formatFileSize(mediaItem.size),
                            style = MaterialTheme.typography.labelSmall,
                            color = Color.White.copy(alpha = 0.8f)
                        )
                        mediaItem.resolution?.let { res ->
                            Text(
                                text = "${res.width}x${res.height}",
                                style = MaterialTheme.typography.labelSmall,
                                color = Color.White.copy(alpha = 0.8f)
                            )
                        }
                    }
                }
            }
        }
    }
}

@Composable
fun LoadingContent() {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center
    ) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            CircularProgressIndicator()
            Text(
                text = "Loading media...",
                style = MaterialTheme.typography.bodyMedium
            )
        }
    }
}

@Composable
fun EmptyContent(
    message: String = "No media found\nMedia will appear here once recorded",
    onRefresh: () -> Unit
) {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center
    ) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            Icon(
                imageVector = Icons.Default.VideoLibrary,
                contentDescription = null,
                modifier = Modifier.size(64.dp),
                tint = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Text(
                text = message,
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                textAlign = androidx.compose.ui.text.style.TextAlign.Center
            )
            Button(onClick = onRefresh) {
                Icon(Icons.Default.Refresh, contentDescription = null)
                Spacer(modifier = Modifier.width(8.dp))
                Text("Refresh")
            }
        }
    }
}

@Composable
fun ErrorContent(message: String, onRetry: () -> Unit) {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center
    ) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(16.dp),
            modifier = Modifier.padding(16.dp)
        ) {
            Icon(
                imageVector = Icons.Default.Error,
                contentDescription = null,
                modifier = Modifier.size(64.dp),
                tint = MaterialTheme.colorScheme.error
            )
            Text(
                text = "Error",
                style = MaterialTheme.typography.titleMedium,
                color = MaterialTheme.colorScheme.error
            )
            Text(
                text = message,
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Button(onClick = onRetry) {
                Text("Retry")
            }
        }
    }
}

// Utility functions
private fun formatDuration(seconds: Int): String {
    val minutes = seconds / 60
    val secs = seconds % 60
    return String.format("%d:%02d", minutes, secs)
}

private fun formatFileSize(bytes: Long): String {
    return when {
        bytes < 1024 -> "$bytes B"
        bytes < 1024 * 1024 -> "${bytes / 1024} KB"
        bytes < 1024 * 1024 * 1024 -> "${bytes / (1024 * 1024)} MB"
        else -> String.format("%.1f GB", bytes / (1024.0 * 1024.0 * 1024.0))
    }
}

private fun formatTimestamp(timestamp: Long): String {
    val sdf = SimpleDateFormat("MMM dd, yyyy HH:mm", Locale.getDefault())
    return sdf.format(Date(timestamp))
}

@Composable
fun VideoPlayerDialog(
    mediaItem: MediaItem,
    onDismiss: () -> Unit,
    getVideoFile: (MediaItem) -> File?
) {
    val videoFile = getVideoFile(mediaItem)
    FileVideoPlayerDialog(
        title = mediaItem.filename,
        videoFile = videoFile,
        onDismiss = onDismiss,
        emptyMessage = "Video file not found. Please download again."
    )
}

@Composable
fun LocalVideoPlayerDialog(
    file: File,
    onDismiss: () -> Unit
) {
    FileVideoPlayerDialog(
        title = file.name,
        videoFile = file,
        onDismiss = onDismiss,
        emptyMessage = "Local video file not available"
    )
}

@OptIn(UnstableApi::class)
@Composable
private fun FileVideoPlayerDialog(
    title: String,
    videoFile: File?,
    onDismiss: () -> Unit,
    emptyMessage: String
) {
    val context = LocalContext.current
    val videoUri = remember(videoFile?.absolutePath) {
        videoFile?.takeIf { it.exists() }?.let { Uri.fromFile(it) }
    }
    var playbackError by remember(videoUri) { mutableStateOf<String?>(null) }
    val exoPlayer = remember(videoUri) {
        videoUri?.let { uri ->
            ExoPlayer.Builder(context).build().apply {
                val mediaItem = ExoMediaItem.fromUri(uri)
                setMediaItem(mediaItem)
                playWhenReady = true
                prepare()
            }
        }
    }

    DisposableEffect(exoPlayer) {
        val player = exoPlayer
        val listener = object : Player.Listener {
            override fun onPlayerError(error: PlaybackException) {
                playbackError = error.localizedMessage ?: "无法播放此视频"
                Toast.makeText(context, "无法播放此视频", Toast.LENGTH_SHORT).show()
            }
        }
        player?.addListener(listener)
        onDispose {
            player?.removeListener(listener)
            player?.release()
        }
    }

    Dialog(
        onDismissRequest = onDismiss,
        properties = DialogProperties(
            dismissOnBackPress = true,
            dismissOnClickOutside = false,
            usePlatformDefaultWidth = false,
            decorFitsSystemWindows = false
        )
    ) {
        Box(
            modifier = Modifier
                .fillMaxSize()
                .background(Color.Black)
        ) {
            when {
                videoUri == null -> {
                    MissingVideoMessage(message = emptyMessage)
                }
                playbackError != null -> {
                    MissingVideoMessage(message = playbackError ?: emptyMessage)
                }
                exoPlayer != null -> {
                    AndroidView(
                        modifier = Modifier.fillMaxSize(),
                        factory = { ctx ->
                            PlayerView(ctx).apply {
                                useController = true
                                player = exoPlayer
                            }
                        },
                        update = { view: PlayerView ->
                            if (view.player !== exoPlayer) {
                                view.player = exoPlayer
                            }
                        }
                    )
                }
                else -> {
                    MissingVideoMessage(message = emptyMessage)
                }
            }

            // Transparent title bar overlay
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .align(Alignment.TopStart)
                    .background(
                        brush = Brush.verticalGradient(
                            colors = listOf(
                                Color.Black.copy(alpha = 0.7f),
                                Color.Transparent
                            )
                        )
                    )
                    .padding(horizontal = 16.dp, vertical = 12.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = title,
                    style = MaterialTheme.typography.titleMedium,
                    color = Color.White,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                    modifier = Modifier.weight(1f)
                )
                IconButton(onClick = onDismiss) {
                    Icon(
                        imageVector = Icons.Default.Close,
                        contentDescription = "Close",
                        tint = Color.White
                    )
                }
            }
        }
    }
}

@Composable
private fun MissingVideoMessage(message: String) {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = message,
            color = Color.White,
            style = MaterialTheme.typography.bodyLarge,
            textAlign = TextAlign.Center,
            modifier = Modifier.padding(24.dp)
        )
    }
}

@Composable
fun LocalRecordingsContent(
    recordings: List<File>,
    onMediaClick: (File) -> Unit,
    onDeleteClick: (File) -> Unit,
    onRefresh: () -> Unit
) {
    if (recordings.isEmpty()) {
        EmptyContent(
            message = "No local recordings yet.\nPress the record button while viewing video to save.",
            onRefresh = onRefresh
        )
    } else {
        LazyVerticalGrid(
            columns = GridCells.Adaptive(minSize = 150.dp),
            contentPadding = PaddingValues(8.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            items(recordings, key = { it.absolutePath }) { file ->
                LocalRecordingCard(
                    file = file,
                    onClick = { onMediaClick(file) },
                    onDeleteClick = { onDeleteClick(file) }
                )
            }
        }
    }
}

@Composable
fun LocalRecordingCard(
    file: File,
    onClick: () -> Unit,
    onDeleteClick: () -> Unit
) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .aspectRatio(16f / 9f)
            .clickable(onClick = onClick),
        elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)
    ) {
        Box {
            // Video thumbnail placeholder
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .background(MaterialTheme.colorScheme.surfaceVariant),
                contentAlignment = Alignment.Center
            ) {
                Icon(
                    Icons.Default.PlayArrow,
                    contentDescription = null,
                    modifier = Modifier.size(48.dp),
                    tint = MaterialTheme.colorScheme.primary
                )
            }
            
            // File info overlay
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .align(Alignment.BottomStart)
                    .background(
                        Brush.verticalGradient(
                            colors = listOf(Color.Transparent, Color.Black.copy(alpha = 0.7f))
                        )
                    )
                    .padding(8.dp)
            ) {
                Text(
                    text = file.name,
                    style = MaterialTheme.typography.bodySmall,
                    color = Color.White,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis
                )
                Text(
                    text = "${file.length() / 1024 / 1024} MB",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color.White.copy(alpha = 0.7f)
                )
            }
            
            // Delete button
            IconButton(
                onClick = onDeleteClick,
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .padding(4.dp)
                    .background(Color.Black.copy(alpha = 0.5f), RoundedCornerShape(50))
            ) {
                Icon(
                    Icons.Default.Delete,
                    contentDescription = "Delete",
                    tint = Color.White
                )
            }
        }
    }
}
