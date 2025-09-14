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
import io.ktor.websocket.CloseReason
import io.ktor.websocket.close
import io.ktor.websocket.readText
import io.ktor.server.response.respondBytes
import io.ktor.http.HttpStatusCode
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import kotlinx.coroutines.TimeoutCancellationException
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
        private const val SEND_TIMEOUT_MS = 75L
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
                        // Serve static assets from the Android assets folder
                        get("/assets/{path...}") {
                            val pathSegments = call.parameters.getAll("path")
                            val assetPath = pathSegments?.joinToString("/") ?: run {
                                call.respondText("Missing asset path", status = HttpStatusCode.BadRequest)
                                return@get
                            }
                            try {
                                val bytes = this@ControlServer.context.assets.open(assetPath).use { it.readBytes() }
                                val ct = when {
                                    assetPath.endsWith(".js", true) -> ContentType.Application.JavaScript
                                    assetPath.endsWith(".css", true) -> ContentType.Text.CSS
                                    assetPath.endsWith(".wasm", true) -> ContentType.parse("application/wasm")
                                    else -> ContentType.Application.OctetStream
                                }
                                call.respondBytes(bytes, ct)
                            } catch (e: Exception) {
                                Log.w(TAG, "Asset not found: $assetPath")
                                call.respondText("Not Found", status = HttpStatusCode.NotFound)
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
        val snapshot = synchronized(connections) { connections.toList() }
        snapshot.forEach { session ->
            sendSafe(session, frame)
        }
    }

    suspend fun broadcastText(text: String) {
        if (connections.isEmpty()) return
        val frame = Frame.Text(text)
        val snapshot = synchronized(connections) { connections.toList() }
        snapshot.forEach { session ->
            sendSafe(session, frame)
        }
    }

    private suspend fun sendSafe(session: io.ktor.websocket.WebSocketSession, frame: Frame) {
        try {
            withTimeout(SEND_TIMEOUT_MS) {
                session.send(frame.copy())
            }
        } catch (t: TimeoutCancellationException) {
            Log.w(TAG, "send timeout; dropping slow client")
            try { session.close(CloseReason(CloseReason.Codes.GOING_AWAY, "send timeout")) } catch (_: Throwable) {}
            connections -= session
        } catch (t: Throwable) {
            Log.w(TAG, "send error; removing client: ${t.message}")
            try { session.close(CloseReason(CloseReason.Codes.UNEXPECTED_CONDITION, "send error")) } catch (_: Throwable) {}
            connections -= session
        }
    }
}
