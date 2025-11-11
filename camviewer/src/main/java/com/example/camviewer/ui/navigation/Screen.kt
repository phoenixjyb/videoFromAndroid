package com.example.camviewer.ui.navigation

sealed class Screen(val route: String) {
    object Video : Screen("video")
    object Media : Screen("media")
    object Settings : Screen("settings")
}
