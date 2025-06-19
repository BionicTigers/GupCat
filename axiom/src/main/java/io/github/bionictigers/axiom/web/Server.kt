package io.github.bionictigers.axiom.web

import com.qualcomm.robotcore.util.RobotLog
import com.squareup.moshi.JsonClass
import com.squareup.moshi.Moshi
import com.squareup.moshi.adapter
import com.squareup.moshi.adapters.PolymorphicJsonAdapterFactory
import com.squareup.moshi.kotlin.reflect.KotlinJsonAdapterFactory
import io.github.bionictigers.axiom.commands.Scheduler
import fi.iki.elonen.NanoWSD
import fi.iki.elonen.NanoHTTPD.IHTTPSession
import fi.iki.elonen.NanoHTTPD.Response
import fi.iki.elonen.NanoHTTPD.Response.Status
import fi.iki.elonen.NanoWSD.WebSocket
import fi.iki.elonen.NanoWSD.WebSocketFrame
import fi.iki.elonen.NanoWSD.WebSocketFrame.CloseCode
import java.io.IOException

@JsonClass(generateAdapter = true)
sealed class IncomingMessage {
    abstract val type: String

    @JsonClass(generateAdapter = true)
    data class Edit(
        override val type: String = "edit",
        val path: String,
        val value: Any
    ) : IncomingMessage()
}

enum class ObjectType {
    Command,
    System
}

object Server {
    // Keep track of active WebSocket connections
    private val connections = mutableListOf<UpdatesWebSocket>()

    // Moshi instance configured with the Kotlin adapter
    private val moshi: Moshi = Moshi.Builder()
        .add(PolymorphicJsonAdapterFactory.of(IncomingMessage::class.java, "type")
            .withSubtype(IncomingMessage.Edit::class.java, "edit"))
        .add(KotlinJsonAdapterFactory())
        .build()

    @OptIn(ExperimentalStdlibApi::class)
    private val anyAdapter = moshi.adapter<Any>()
    @OptIn(ExperimentalStdlibApi::class)
    private val messageAdapter = moshi.adapter<IncomingMessage>()

    @JvmStatic
    internal fun start() {
        println("Start Called")
        // Create & start the NanoWSD server on port 10464
        val server = object : NanoWSD(10464) {
            override fun openWebSocket(handshake: IHTTPSession): WebSocket {
                // Called when a new WebSocket client connects to any path.
                return UpdatesWebSocket(handshake)
            }

            override fun serveHttp(session: IHTTPSession): Response {
                // Fallback for non-WebSocket HTTP requests.
                return newFixedLengthResponse(Status.OK, "text/plain", "NanoHTTPD on /")
            }
        }
        server.start()
        RobotLog.dd("Axiom", "NanoWSD server started on port 10464")

        // Background thread to periodically broadcast updates from Scheduler
        Thread {
            while (true) {
                Thread.sleep(100)  // simulate periodic updates
                val updates = serialize()
                val updateJson = try {
                    anyAdapter.toJson(updates)
                } catch (e: Exception) {
                    RobotLog.dd("Axiom", "Serialization error: ${e.message}")
                    "{}"
                }
                synchronized(connections) {
                    connections.forEach { ws ->
                        ws.sendSafe(updateJson)
                    }
                }
            }
        }.start()
    }

    private fun serialize(): Map<String, Any?> {
        return mapOf(
            "type" to "scheduler_update",
            "data" to Scheduler.serialize(),
        )
    }

    // Custom WebSocket subclass for handling connect/close/message events
    private class UpdatesWebSocket(handshakeRequest: IHTTPSession) : WebSocket(handshakeRequest) {
        override fun onOpen() {
            RobotLog.dd("Axiom", "Client connected: ${handshakeRequest.remoteIpAddress}")
            synchronized(connections) {
                connections.add(this)
            }
        }

        override fun onClose(
            code: CloseCode?,
            reason: String?,
            initiatedByRemote: Boolean
        ) {
            RobotLog.dd("Axiom", "Client disconnected: ${handshakeRequest.remoteIpAddress} (code=$code, reason=$reason)")
            synchronized(connections) {
                connections.remove(this)
            }
        }

        override fun onMessage(message: WebSocketFrame?) {
            message?.let {
                val clientMsg = it.textPayload
                RobotLog.dd("Axiom", "Received from client: $clientMsg")

                if (clientMsg == "ping") return

                try {
                    when (val msg = messageAdapter.fromJson(clientMsg)) {
                        is IncomingMessage.Edit -> {
                            println("Edit message received:")
                            println("Path: ${msg.path} to ${msg.value}")

                            Scheduler.edit(msg.path, msg.value)
                        }
                        null -> {
                            RobotLog.dd("Axiom", "Received null after parsing the message")
                        }
                    }
                } catch (e: Exception) {
                    RobotLog.ww("Axiom", "Failed to parse client message: ${e.message}")
                }
            }
        }

        override fun onPong(pong: WebSocketFrame?) {
            // Log pong responses, if needed.
            RobotLog.dd("Axiom", "Received pong from ${handshakeRequest.remoteIpAddress}")
        }

        override fun onException(exception: IOException?) {
            RobotLog.ww("Axiom", "WebSocket Exception: ${exception?.message}")
            // Gracefully close the connection when an exception occurs.
            try {
                close(CloseCode.InternalServerError, "Handler terminated due to exception", false)
            } catch (e: Exception) {
                RobotLog.ww("Axiom", "Failed to close WebSocket: ${e.message}")
            }
        }

        /** Helper method to send text safely, ignoring exceptions */
        fun sendSafe(text: String) {
            try {
                send(text)
            } catch (e: IOException) {
                RobotLog.ww("Axiom", "Failed to send to client: ${e.message}")
            }
        }
    }
}
