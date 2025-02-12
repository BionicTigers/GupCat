package org.firstinspires.ftc.teamcode.input

import com.qualcomm.robotcore.hardware.Gamepad as FTCGamepad
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.commands.*

interface GamepadSystemState : CommandState {
    val gamepads: ArrayList<Gamepad>

    companion object {
        fun default(gamepads: Pair<Gamepad, Gamepad>): GamepadSystemState {
            val (gamepad1, gamepad2) = gamepads

            return object : GamepadSystemState, CommandState by CommandState.default("GamepadSystem") {
                override val gamepads = arrayListOf(gamepad1, gamepad2)
            }
        }
    }
}


class GamepadSystem(gamepad1FTC: FTCGamepad, gamepad2FTC: FTCGamepad) : System {
    companion object {
        var activeSystem: GamepadSystem? = null
            private set
    }

    val gamepads = Pair(Gamepad(gamepad1FTC, this), Gamepad(gamepad2FTC, this))

    override val beforeRun = Command(GamepadSystemState.default(gamepads))
    override val afterRun = null

    override val dependencies: List<System> = emptyList()

    init {
        val (gamepad1, gamepad2) = gamepads
        gamepad1.command.dependsOn(beforeRun)
        gamepad2.command.dependsOn(beforeRun)

        Scheduler.add(gamepad1.command, gamepad2.command)

        activeSystem = this
    }
}