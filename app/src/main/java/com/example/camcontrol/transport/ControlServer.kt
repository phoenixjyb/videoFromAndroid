package com.example.camcontrol.transport

import android.content.Context
import android.util.Log
import io.ktor.serialization.kotlinx.json.json
import io.ktor.server.application.call
import io.ktor.server.application.install
import io.ktor.server.cio.CIO
import io.ktor.server.engine.ApplicationEngine
import io.ktor.server.engine.embeddedServer
import io.ktor.server.plugins.contentnegotiation.ContentNegotiation
import io.ktor.server.response.respondText
import io.ktor.server.routing.get
import io.ktor.server.routing.routing
import io.ktor.http.ContentType
import io.ktor.server.websocket.WebSockets
import io.ktor.server.websocket.webSocket
import io.ktor.websocket.Frame
import io.ktor.websocket.readText
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.util.Collections

class ControlServer(
    private val context: Context,
    private val scope: CoroutineScope,
    private val onCommandReceived: (String) -> Unit
) {
    private var server: ApplicationEngine? = null
    private val connections = Collections.synchronizedSet<io.ktor.websocket.WebSocketSession>(LinkedHashSet())

    companion object {
        private const val TAG = "ControlServer"
        private const val PORT = 9090
    }

    fun start() {
        if (server != null) {
            Log.w(TAG, "Server already started")
            return
        }
        Log.i(TAG, "Starting WebSocket server on port $PORT...")
        scope.launch(Dispatchers.IO) {
            try {
                // Stop any existing server first
                server?.stop(0, 5000)
                server = null
                
                server = embeddedServer(CIO, host = "0.0.0.0", port = PORT) {
                    install(WebSockets)
                    install(ContentNegotiation) { json() }
                    routing {
                        get("/") {
                            Log.i(TAG, "📥 HTTP GET request received for /")
                            try {
                                val inputStream = this@ControlServer.context.assets.open("index.html")
                                val htmlContent = inputStream.bufferedReader().use { it.readText() }
                                Log.i(TAG, "📄 Serving HTML content (${htmlContent.length} bytes)")
                                call.respondText(htmlContent, ContentType.Text.Html)
                            } catch (e: Exception) {
                                Log.e(TAG, "Failed to load HTML: ${e.message}")
                                call.respondText("Server is running. WebSocket endpoint: /control", ContentType.Text.Plain)
                            }
                        }
                        webSocket("/control") {
                            connections += this
                            Log.i(TAG, "✅ New client connected: ${connections.size} total")
                            try {
                                for (frame in incoming) {
                                    if (frame is Frame.Text) {
                                        val command = frame.readText()
                                        Log.d(TAG, "📤 Command received: $command")
                                        onCommandReceived(command)
                                    }
                                }
                            } catch (e: Exception) {
                                Log.i(TAG, "Client disconnected: ${e.message}")
                            } finally {
                                connections -= this
                                Log.i(TAG, "Client removed: ${connections.size} total")
                            }
                        }
                    }
                }.also { 
                    it.start(wait = false)
                    Log.i(TAG, "🚀 WebSocket server started successfully on port $PORT")
                }
            } catch (e: Exception) {
                Log.e(TAG, "❌ Failed to start WebSocket server", e)
                server = null
            }
        }
    }

    fun stop() {
        Log.d(TAG, "Stopping server.")
        try {
            server?.stop(1000, 2000)
        } catch (e: Exception) {
            Log.w(TAG, "Error stopping server: ${e.message}")
        } finally {
            server = null
            connections.clear()
        }
    }

    suspend fun broadcastVideoData(data: ByteArray) {
        if (connections.isEmpty()) return
        val frame = Frame.Binary(true, data)
        connections.forEach {
            it.send(frame.copy())
        }
    }

    suspend fun broadcastText(text: String) {
        if (connections.isEmpty()) return
        val frame = Frame.Text(text)
        connections.forEach {
            it.send(frame.copy())
        }
    }
}
