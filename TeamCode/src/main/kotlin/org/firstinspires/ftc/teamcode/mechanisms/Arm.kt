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

class Arm(hardwareMap: HardwareMap, telemetry: Telemetry? = null) : System, Controllable {
    enum class Position(val target: Double) {
        Down(.97),
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

    fun up(): BaseCommand = Command.instant("Arm Up") { target = Position.Up }

    fun down(): BaseCommand = Command.instant("Arm Down") { target = Position.Down }

    fun middle(): BaseCommand = Command.instant("Arm Middle") { target = Position.Middle }

    fun toggle(): BaseCommand = Command.instant("Arm Toggle") {
        target = if (target != Position.Down) {
            Position.Down
        } else {
            Position.Up
        }
    }

    init {
        if (telemetry != null) {
            Scheduler.schedule(Command.continuous("Arm Log") {
                telemetry.addData("Arm Position", target.name)
            })
        }
    }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit = with(profile.arm) {
        if (!gamepad.matches(desiredGamepad)) return@with

        toggleUpDown?.let { builder.register(it) { toggle() } }
        up?.let { builder.register(it) { up() } }
        middle?.let { builder.register(it) { middle() } }
        down?.let { builder.register(it) { down() } }
    }
}