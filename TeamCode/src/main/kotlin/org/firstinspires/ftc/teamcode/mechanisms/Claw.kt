package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.commands.BaseCommand
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.input.ControlSchema
import org.firstinspires.ftc.teamcode.input.Controllable
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.Gamepads
import org.firstinspires.ftc.teamcode.input.Profile
import org.firstinspires.ftc.teamcode.input.matches
import org.firstinspires.ftc.teamcode.input.types.Digital
import org.firstinspires.ftc.teamcode.utils.getByName

class Claw(hardwareMap: HardwareMap, telemetry: Telemetry? = null) : System, Controllable {
    companion object {
        /** Open position for the servo */
        const val OPEN_POSITION = .1

        /** Close position for the servo */
        const val CLOSE_POSITION = 0.55
    }

    interface Schema : ControlSchema {
        /** Opens the claw */
        val open: Digital?

        /** Closes the claw */
        val close: Digital?

        /** Toggles the claw */
        val toggle: Digital?
    }

    override val name = "Claw"

    private val claw = hardwareMap.getByName<Servo>("claw")
    private var open = false

    fun open(): BaseCommand = Command.instant("Claw Open") {
        open = true
        claw.position = OPEN_POSITION
    }

    fun close(): BaseCommand = Command.instant("Claw Close") {
        open = false
        claw.position = CLOSE_POSITION
    }

    fun toggle(): BaseCommand = Command.instant("Claw Toggle") {
        open = !open
        claw.position = if (open) OPEN_POSITION else CLOSE_POSITION
    }

    init {
        if (telemetry != null) {
            Scheduler.schedule(Command.continuous("Claw Log") {
                telemetry.addData("Claw Position", claw.position)
            })
        }
    }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.claw) {
            if (!gamepad.matches(desiredGamepad)) return@with

            open?.let { builder.register(it) { open() } }
            close?.let { builder.register(it) { close() } }
            toggle?.let { builder.register(it) { toggle() } }
        }
}