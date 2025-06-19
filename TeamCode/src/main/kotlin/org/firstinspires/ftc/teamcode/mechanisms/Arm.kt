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

class Arm(hardwareMap: HardwareMap) : System, Controllable {
    enum class Position(val target: Double) {
        Down(1.0),
        Middle(.65),
        Up(.15),
    }

    interface Schema : ControlSchema {
        /** Toggle between up and down */
        val toggleUpDown: Digital?

        /** Move to up position */
        val up: Digital?

        /** Move to middle position */
        val middle: Digital?

        /** Move to down position */
        val down: Digital?
    }

    override val name = "Arm"

    private val arm = hardwareMap.get(Servo::class.java, "arm")

    var target = Position.Up
        private set(value) {
            arm.position = value.target
            field = value
        }

    fun up(): InstantCommand = InstantCommand { target = Position.Up }

    fun down(): InstantCommand = InstantCommand { target = Position.Down }

    fun middle(): InstantCommand = InstantCommand { target = Position.Middle }

    fun toggle(): InstantCommand = InstantCommand {
        if (target != Position.Down) {
            target = Position.Down
        } else {
            target = Position.Up
        }
    }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit = with(profile.arm) {
        toggleUpDown?.let { builder.register(it) { toggle() } }
        up?.let { builder.register(it) { up() } }
        middle?.let { builder.register(it) { middle() } }
        down?.let { builder.register(it) { down() } }
    }
}