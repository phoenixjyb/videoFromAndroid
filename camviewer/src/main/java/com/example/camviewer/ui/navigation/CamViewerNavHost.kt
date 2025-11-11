package com.example.camviewer.ui.navigation

import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.VideoLibrary
import androidx.compose.material.icons.filled.Videocam
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.unit.dp
import androidx.navigation.NavDestination.Companion.hierarchy
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import com.example.camviewer.ui.screens.media.MediaScreen
import com.example.camviewer.ui.screens.settings.SettingsScreen
import com.example.camviewer.ui.screens.video.VideoScreen

data class NavigationItem(
    val screen: Screen,
    val icon: ImageVector,
    val label: String
)

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun CamViewerNavHost() {
    val navController = rememberNavController()
    val navBackStackEntry by navController.currentBackStackEntryAsState()
    val currentDestination = navBackStackEntry?.destination
    
    val navigationItems = listOf(
        NavigationItem(Screen.Video, Icons.Default.Videocam, "Video"),
        NavigationItem(Screen.Media, Icons.Default.VideoLibrary, "Media"),
        NavigationItem(Screen.Settings, Icons.Default.Settings, "Settings")
    )
    
    Row(modifier = Modifier.fillMaxSize()) {
        // Main content
        Box(modifier = Modifier.weight(1f)) {
            NavHost(
                navController = navController,
                startDestination = Screen.Video.route,
                modifier = Modifier.fillMaxSize()
            ) {
                composable(Screen.Video.route) {
                    VideoScreen()
                }
                composable(Screen.Media.route) {
                    MediaScreen()
                }
                composable(Screen.Settings.route) {
                    SettingsScreen()
                }
            }
        }
        
        // Vertical navigation rail on the right
        NavigationRail(
            modifier = Modifier.fillMaxHeight(),
            containerColor = MaterialTheme.colorScheme.surface.copy(alpha = 0.95f)
        ) {
            Spacer(modifier = Modifier.weight(1f))
            
            navigationItems.forEach { item ->
                NavigationRailItem(
                    icon = { Icon(item.icon, contentDescription = item.label, modifier = Modifier.size(20.dp)) },
                    label = { Text(item.label, style = MaterialTheme.typography.labelSmall) },
                    selected = currentDestination?.hierarchy?.any { it.route == item.screen.route } == true,
                    onClick = {
                        navController.navigate(item.screen.route) {
                            popUpTo(navController.graph.findStartDestination().id) {
                                saveState = true
                            }
                            launchSingleTop = true
                            restoreState = true
                        }
                    },
                    alwaysShowLabel = false
                )
            }
            
            Spacer(modifier = Modifier.weight(1f))
        }
    }
}
