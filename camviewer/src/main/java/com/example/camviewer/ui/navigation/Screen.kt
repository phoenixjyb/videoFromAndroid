package com.example.camviewer.ui.navigation

sealed class Screen(val route: String) {
    object Video : Screen("video")
    object Media : Screen("media")
    object OrinControl : Screen("orin_control")
    object Settings : Screen("settings")
}
