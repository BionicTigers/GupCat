package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.commands.InstantCommand
import io.github.bionictigers.axiom.commands.System
import org.firstinspires.ftc.teamcode.input.ControlSchema
import org.firstinspires.ftc.teamcode.input.Controllable
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.Gamepads
import org.firstinspires.ftc.teamcode.input.Profile
import org.firstinspires.ftc.teamcode.input.types.Digital
import org.firstinspires.ftc.teamcode.utils.getByName

class Claw(hardwareMap: HardwareMap) : System, Controllable {
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

    fun open(): InstantCommand = InstantCommand { claw.position = OPEN_POSITION }
    fun close(): InstantCommand = InstantCommand { claw.position = CLOSE_POSITION }
    fun toggle(): InstantCommand = InstantCommand {
        claw.position = if (claw.position == OPEN_POSITION) CLOSE_POSITION else OPEN_POSITION
    }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.claw) {
            open?.let { builder.register(it) { open() } }
            close?.let { builder.register(it) { close() } }
            toggle?.let { builder.register(it) { toggle() } }
        }
}