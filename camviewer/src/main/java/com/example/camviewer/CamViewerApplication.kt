package com.example.camviewer

import android.app.Application
import dagger.hilt.android.HiltAndroidApp

@HiltAndroidApp
class CamViewerApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        // Application initialization
    }
}
