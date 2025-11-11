package com.example.camviewer.di

import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import io.ktor.client.*
import io.ktor.client.engine.cio.*
import io.ktor.client.plugins.*
import io.ktor.client.plugins.contentnegotiation.*
import io.ktor.client.plugins.logging.*
import io.ktor.client.plugins.websocket.*
import io.ktor.serialization.kotlinx.json.*
import kotlinx.serialization.json.Json
import javax.inject.Qualifier
import javax.inject.Singleton

@Qualifier
@Retention(AnnotationRetention.BINARY)
annotation class KtorHttpClient

@Qualifier
@Retention(AnnotationRetention.BINARY)
annotation class KtorWebSocketClient

@Module
@InstallIn(SingletonComponent::class)
object NetworkModule {
    
    @Provides
    @Singleton
    fun provideJson(): Json = Json {
        ignoreUnknownKeys = true
        prettyPrint = true
        isLenient = true
        encodeDefaults = true
    }
    
    @Provides
    @Singleton
    @KtorHttpClient
    fun provideHttpClient(json: Json): HttpClient = HttpClient(CIO) {
        install(ContentNegotiation) {
            json(json)
        }
        
        install(Logging) {
            logger = Logger.DEFAULT
            level = LogLevel.INFO
        }
        
        install(HttpTimeout) {
            requestTimeoutMillis = 30_000
            connectTimeoutMillis = 10_000
            socketTimeoutMillis = 30_000
        }
        
        // Support for partial content (resume downloads)
        install(HttpRequestRetry) {
            retryOnServerErrors(maxRetries = 3)
            exponentialDelay()
        }
    }
    
    @Provides
    @Singleton
    @KtorWebSocketClient
    fun provideWebSocketClient(json: Json): HttpClient = HttpClient(CIO) {
        install(WebSockets) {
            pingInterval = 20_000
            maxFrameSize = 10 * 1024 * 1024 // 10MB for video frames
        }
        
        install(ContentNegotiation) {
            json(json)
        }
        
        install(Logging) {
            logger = Logger.DEFAULT
            level = LogLevel.INFO
        }
        
        install(HttpTimeout) {
            requestTimeoutMillis = 60_000
            connectTimeoutMillis = 10_000
        }
    }
}
