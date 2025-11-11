package com.example.camviewer.data.repository

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import com.example.camviewer.data.model.AppSettings
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
class SettingsRepository @Inject constructor(
    private val dataStore: DataStore<Preferences>
) {
    private object PreferencesKeys {
        val CAMERA_URL = stringPreferencesKey("camera_url")
        val ORIN_TARGET_URL = stringPreferencesKey("orin_target_url")
        val ORIN_MEDIA_URL = stringPreferencesKey("orin_media_url")
        val DEVELOPER_MODE = booleanPreferencesKey("developer_mode")
    }
    
    val settings: Flow<AppSettings> = dataStore.data.map { preferences ->
        AppSettings(
            cameraUrl = preferences[PreferencesKeys.CAMERA_URL] ?: "ws://192.168.1.100:9090",
            orinTargetUrl = preferences[PreferencesKeys.ORIN_TARGET_URL] ?: "http://192.168.1.200:8080",
            orinMediaUrl = preferences[PreferencesKeys.ORIN_MEDIA_URL] ?: "http://192.168.1.200:8081",
            developerModeEnabled = preferences[PreferencesKeys.DEVELOPER_MODE] ?: false
        )
    }
    
    suspend fun updateSettings(settings: AppSettings) {
        dataStore.edit { preferences ->
            preferences[PreferencesKeys.CAMERA_URL] = settings.cameraUrl
            preferences[PreferencesKeys.ORIN_TARGET_URL] = settings.orinTargetUrl
            preferences[PreferencesKeys.ORIN_MEDIA_URL] = settings.orinMediaUrl
            preferences[PreferencesKeys.DEVELOPER_MODE] = settings.developerModeEnabled
        }
    }
    
    suspend fun setDeveloperMode(enabled: Boolean) {
        dataStore.edit { preferences ->
            preferences[PreferencesKeys.DEVELOPER_MODE] = enabled
        }
    }
}
