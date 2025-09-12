package com.example.camcontrol.transport

import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.PrintWriter
import java.net.ServerSocket
import java.net.Socket

class SimpleControlServer(
    private val scope: CoroutineScope,
    private val onCommandReceived: (String) -> Unit
) {
    private var serverSocket: ServerSocket? = null
    private val connections = mutableListOf<Socket>()

    companion object {
        private const val TAG = "SimpleControlServer"
        private const val PORT = 8080
    }

    fun start() {
        if (serverSocket != null) {
            Log.w(TAG, "Server already started")
            return
        }
        
        Log.i(TAG, "🚀 Starting simple server on port $PORT...")
        scope.launch(Dispatchers.IO) {
            try {
                serverSocket = ServerSocket(PORT)
                Log.i(TAG, "✅ Server started successfully on port $PORT")
                
                while (true) {
                    val client = serverSocket?.accept() ?: break
                    Log.i(TAG, "📱 Client connected: ${client.remoteSocketAddress}")
                    
                    scope.launch(Dispatchers.IO) {
                        handleClient(client)
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "❌ Server error", e)
            }
        }
    }
    
    private fun handleClient(client: Socket) {
        try {
            connections.add(client)
            val input = BufferedReader(InputStreamReader(client.getInputStream()))
            val output = PrintWriter(client.getOutputStream(), true)
            
            // Read the HTTP request
            val requestLine = input.readLine()
            Log.d(TAG, "📤 Request: $requestLine")
            
            // Skip headers
            while (true) {
                val line = input.readLine()
                if (line.isNullOrEmpty()) break
            }
            
            // Send proper HTTP response
            output.println("HTTP/1.1 200 OK")
            output.println("Content-Type: text/plain")
            output.println("Content-Length: 25")
            output.println("Connection: close")
            output.println()
            output.println("CamControl Server - OK")
            output.flush()
            
            Log.d(TAG, "✅ HTTP response sent")
            
        } catch (e: Exception) {
            Log.i(TAG, "Client disconnected: ${e.message}")
        } finally {
            connections.remove(client)
            try {
                client.close()
            } catch (e: Exception) {
                Log.w(TAG, "Error closing client", e)
            }
        }
    }

    fun stop() {
        Log.d(TAG, "Stopping server")
        try {
            connections.forEach { it.close() }
            connections.clear()
            serverSocket?.close()
            serverSocket = null
        } catch (e: Exception) {
            Log.w(TAG, "Error stopping server", e)
        }
    }
}
