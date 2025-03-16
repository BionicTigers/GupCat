package io.github.bionictigers.axiom.web

import android.annotation.SuppressLint
import android.content.Context
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier
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
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.ftccommon.external.OnCreate
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndClass
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes
import java.io.IOException
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.ConcurrentLinkedQueue
import kotlin.coroutines.CoroutineContext
import kotlin.coroutines.coroutineContext

@JsonClass(generateAdapter = true)
sealed class IncomingMessage {
    abstract val type: String

    @JsonClass(generateAdapter = true)
    data class Edit(
        override val type: String = "edit",
        val path: String,
        val value: Any
    ) : IncomingMessage()

    @JsonClass(generateAdapter = true)
    data class OpMode(
        override val type: String = "opModeUpdate",
        val opMode: String,
        val state: String
    ) : IncomingMessage()
}

object Server {
    // Keep track of active WebSocket connections
    private val connections = mutableListOf<UpdatesWebSocket>()

    private val serverJob = SupervisorJob()
    private val serverScope = CoroutineScope(Dispatchers.Default + serverJob)

    private var isRunning = false

    // Moshi instance configured with the Kotlin adapter
    private val moshi: Moshi = Moshi.Builder()
        .add(PolymorphicJsonAdapterFactory.of(IncomingMessage::class.java, "type")
            .withSubtype(IncomingMessage.Edit::class.java, "edit")
            .withSubtype(IncomingMessage.OpMode::class.java, "opModeUpdate"))
        .add(KotlinJsonAdapterFactory())
        .build()

    @OptIn(ExperimentalStdlibApi::class)
    private val anyAdapter = moshi.adapter<Any>()
    @OptIn(ExperimentalStdlibApi::class)
    private val messageAdapter = moshi.adapter<IncomingMessage>()

    @JvmStatic
    @OnCreate
    fun start(context: Context?) {
        println("Axiom Starting")
        if (isRunning) return
        isRunning = true

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

        //Scheduler update loop
        serverScope.launch {
            while (true) {
                val data = generateSchedulerData()
                send(data)
                delay(100)
            }
        }
    }

    private fun send(connection: UpdatesWebSocket, data: Map<String, Any?>) {
        val serializedJson = try {
            anyAdapter.toJson(data)
        } catch (e: Exception) {
            RobotLog.dd("Axiom", "Serialization error: ${e.message}")
            "{}"
        }
        synchronized(connections) {
            connection.sendSafe(serializedJson)
        }
    }

    private fun send(data: Map<String, Any?>) {
        val serializedJson = try {
            anyAdapter.toJson(data)
        } catch (e: Exception) {
            RobotLog.dd("Axiom", "Serialization error: ${e.message}")
            "{}"
        }
        synchronized(connections) {
            connections.forEach { ws ->
                ws.sendSafe(serializedJson)
            }
        }
    }

    private fun generateSchedulerData(): Map<String, Any?> {
        return mapOf(
            "type" to "cycle",
            "commands" to Scheduler.serialize(),
            "drivetrain" to WebData.drivetrain,
        )
    }

    private fun generateOpModeList(teleOpModes: List<String>, autoOpMode: List<String>): Map<String, Any?> {
        return mapOf(
            "type" to "opModeList",
            "teleops" to teleOpModes,
            "autos" to autoOpMode
        )
    }

    private fun generateOpModeChange(opModeName: String?, opModeState: FTCLayer.OpModeState): Map<String, Any?> {
        return mapOf(
            "type" to "opMode",
            "selected" to opModeName,
            "state" to opModeState.name
        )
    }

    // Custom WebSocket subclass for handling connect/close/message events
    private class UpdatesWebSocket(handshakeRequest: IHTTPSession) : WebSocket(handshakeRequest) {
        override fun onOpen() {
            RobotLog.dd("Axiom", "Client connected: ${handshakeRequest.remoteIpAddress}")
            synchronized(connections) {
                connections.add(this)
                if (FTCLayer.teleOpModes.isNotEmpty() || FTCLayer.autoOpModes.isNotEmpty())
                    send(this, generateOpModeList(FTCLayer.teleOpModes.toList(), FTCLayer.autoOpModes.toList()))
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
                if (clientMsg == "ping") return

                RobotLog.dd("Axiom", clientMsg )

                try {
                    when (val msg = messageAdapter.fromJson(clientMsg)) {
                        is IncomingMessage.Edit -> {
                            Scheduler.edit(msg.path, msg.value)
                        }
                        is IncomingMessage.OpMode -> {
                            if (FTCLayer.opModeState == FTCLayer.OpModeState.Selected && msg.state == "Init") FTCLayer.initOpMode(msg.opMode)
                            else if (FTCLayer.opModeState == FTCLayer.OpModeState.Init && msg.state == "Start") FTCLayer.startOpMode()
                            else if (FTCLayer.opModeState == FTCLayer.OpModeState.Running && msg.state == "Stop") FTCLayer.stopOpMode()
                            else RobotLog.ww("Axiom", "Unknown opmode request - " + FTCLayer.opModeState)
                        }
                        null -> {
                            RobotLog.ww("Axiom", "Received null after parsing the message")
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

    object FTCLayer : OpModeManagerNotifier.Notifications {
        enum class OpModeState {
            Selected,
            Init,
            Running
        }

        data class FTCApplicationData(var eventLoop: FtcEventLoop? = null, var opModeManager: OpModeManagerImpl? = null)

        private val applicationData = FTCApplicationData()
        var eventLoop: FtcEventLoop?
            get() = applicationData.eventLoop
            set(value) {
                applicationData.eventLoop = value
            }
        var opModeManager: OpModeManagerImpl?
            get() = applicationData.opModeManager
            set(value) {
                applicationData.opModeManager = value
            }

        val teleOpModes = ConcurrentLinkedQueue<String>()
        val autoOpModes = ConcurrentLinkedQueue<String>()

        val selectedOpMode: OpMode?
            get() = opModeManager?.activeOpMode
        val selectedOpModeName: String?
            get() = opModeManager?.activeOpModeName.takeUnless { it == "\$Stop\$Robot\$" }
        var opModeState: OpModeState = OpModeState.Selected
            private set

        @JvmStatic
        @OnCreateEventLoop
        fun eventLoopHandler(context: Context, eventLoop: FtcEventLoop) {
            RobotLog.dd("Axiom", "Event Loop")
            this.eventLoop = eventLoop

            opModeManager?.unregisterListener(this)
            RegisteredOpModes.getInstance()

            opModeManager = eventLoop.opModeManager
            opModeManager?.registerListener(this)

            serverScope.launch {
                val registeredOpModes = RegisteredOpModes.getInstance()
                RobotLog.dd("Axiom", "Waiting for op modes")
                registeredOpModes.waitOpModesRegistered()
                RobotLog.dd("Axiom", "Waiting finished")
                RobotLog.dd("Axiom", registeredOpModes.opModes.toString())

                registeredOpModes.opModes.filter { it.flavor != OpModeMeta.Flavor.SYSTEM }.forEach {
                    if (it.flavor == OpModeMeta.Flavor.TELEOP) teleOpModes.add(it.name)
                    if (it.flavor == OpModeMeta.Flavor.AUTONOMOUS) autoOpModes.add(it.name)
                }
                send(generateOpModeList(teleOpModes.toList(), autoOpModes.toList()))
            }
        }

        fun initOpMode(opModeName: String) {
            if (!teleOpModes.contains(opModeName) && !autoOpModes.contains(opModeName)) {
                RobotLog.ww("Axiom", "Cannot start opMode as it doesn't exist")
                return
            }

            opModeManager?.initOpMode(opModeName)
        }

        fun startOpMode() {
            opModeManager?.startActiveOpMode()
        }

        fun stopOpMode() {
            opModeManager?.stopActiveOpMode()
        }

        override fun onOpModePreInit(opMode: OpMode?) {
            Scheduler.clear()
            if (selectedOpModeName == null) return
            opModeState = OpModeState.Init
            send(generateOpModeChange(selectedOpModeName, opModeState))
        }

        override fun onOpModePreStart(opMode: OpMode?) {
            if (selectedOpModeName == null) return
            opModeState = OpModeState.Running
            send(generateOpModeChange(selectedOpModeName, opModeState))
        }

        override fun onOpModePostStop(opMode: OpMode?) {
            Scheduler.clear()
            if (selectedOpModeName == null) return
            opModeState = OpModeState.Selected
            send(generateOpModeChange(selectedOpModeName, opModeState))
        }
    }
}
