package com.example.camviewer.data.repository

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import com.example.camviewer.data.model.AppSettings
import com.example.camviewer.data.model.NetworkPreset
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
class SettingsRepository @Inject constructor(
    private val dataStore: DataStore<Preferences>
) {
    private object PreferencesKeys {
        val NETWORK_PRESET = stringPreferencesKey("network_preset")
        val CAMERA_URL = stringPreferencesKey("camera_url")
        val ORIN_TARGET_URL = stringPreferencesKey("orin_target_url")
        val ORIN_MEDIA_URL = stringPreferencesKey("orin_media_url")
        val PHONE_CONTROL_HOST = stringPreferencesKey("phone_control_host")
        val DEVELOPER_MODE = booleanPreferencesKey("developer_mode")
    }
    
    val settings: Flow<AppSettings> = dataStore.data.map { preferences ->
        val presetName = preferences[PreferencesKeys.NETWORK_PRESET] ?: NetworkPreset.ZEROTIER.name
        val preset = NetworkPreset.fromName(presetName)
        
        AppSettings(
            networkPreset = preset,
            cameraUrl = preferences[PreferencesKeys.CAMERA_URL] ?: preset.getPhoneVideoUrl(),
            orinTargetUrl = preferences[PreferencesKeys.ORIN_TARGET_URL] ?: preset.getOrinTargetUrl(),
            orinMediaUrl = preferences[PreferencesKeys.ORIN_MEDIA_URL] ?: preset.getOrinMediaUrl(),
            phoneControlHost = preferences[PreferencesKeys.PHONE_CONTROL_HOST] ?: preset.phoneIp,
            developerModeEnabled = preferences[PreferencesKeys.DEVELOPER_MODE] ?: false
        )
    }
    
    suspend fun updateSettings(settings: AppSettings) {
        dataStore.edit { preferences ->
            preferences[PreferencesKeys.NETWORK_PRESET] = settings.networkPreset.name
            preferences[PreferencesKeys.CAMERA_URL] = settings.cameraUrl
            preferences[PreferencesKeys.ORIN_TARGET_URL] = settings.orinTargetUrl
            preferences[PreferencesKeys.ORIN_MEDIA_URL] = settings.orinMediaUrl
            preferences[PreferencesKeys.PHONE_CONTROL_HOST] = settings.phoneControlHost
            preferences[PreferencesKeys.DEVELOPER_MODE] = settings.developerModeEnabled
        }
    }
    
    suspend fun applyNetworkPreset(preset: NetworkPreset) {
        dataStore.edit { preferences ->
            preferences[PreferencesKeys.NETWORK_PRESET] = preset.name
            if (preset != NetworkPreset.CUSTOM) {
                preferences[PreferencesKeys.CAMERA_URL] = preset.getPhoneVideoUrl()
                preferences[PreferencesKeys.ORIN_TARGET_URL] = preset.getOrinTargetUrl()
                preferences[PreferencesKeys.ORIN_MEDIA_URL] = preset.getOrinMediaUrl()
                preferences[PreferencesKeys.PHONE_CONTROL_HOST] = preset.phoneIp
            }
        }
    }
    
    suspend fun setDeveloperMode(enabled: Boolean) {
        dataStore.edit { preferences ->
            preferences[PreferencesKeys.DEVELOPER_MODE] = enabled
        }
    }
}
