package org.firstinspires.ftc.teamcode.input

import com.qualcomm.robotcore.hardware.Gamepad
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
import org.firstinspires.ftc.teamcode.input.types.Analog
import org.firstinspires.ftc.teamcode.input.types.Digital

typealias DigitalCommand = (Boolean) -> Command<*>
typealias AnalogCommand = (Double) -> Command<*>

class Controls(
    val gamepad1: Gamepad,
    val gamepad2: Gamepad,
    val profile1: Profile,
    val profile2: Profile,
    controllables: List<Controllable>
) : System {
    override val name = "controls"

    val digitalGP1: Map<Digital, DigitalCommand>
    val analogGP1: Map<Analog, AnalogCommand>
    val digitalGP2: Map<Digital, DigitalCommand>
    val analogGP2: Map<Analog, AnalogCommand>

    init {
        val builderGP1 = Builder()
        val builderGP2 = Builder()

        controllables.forEach {
            it.bindControls(profile1, Gamepads.GAMEPAD_1, builderGP1)
            it.bindControls(profile2, Gamepads.GAMEPAD_2, builderGP2)
        }

        digitalGP1 = builderGP1.digitalControls
        analogGP1 = builderGP1.analogControls
        digitalGP2 = builderGP2.digitalControls
        analogGP2 = builderGP2.analogControls
    }

    private fun updateDigitalControls(gamepad: Gamepad, controls: Map<Digital, DigitalCommand>) {
        controls.forEach { (control, command) ->
            val value = control.get(gamepad)

            when (control.type) {
                Digital.ControlType.PRESS ->
                    if (value && !control.previousValue) {
                        Scheduler.schedule(command(true))
                        println("press")
                    }
                Digital.ControlType.RELEASE ->
                    if (!value && control.previousValue) {
                        Scheduler.schedule(command(false))
                    }
                Digital.ControlType.HOLD ->
                    if (value) {
                        Scheduler.schedule(command(true))
                    }
            }

            control.previousValue = value
        }
    }

    private fun updateAnalogControls(gamepad: Gamepad, controls: Map<Analog, AnalogCommand>) {
        controls.forEach { (control, command) ->
            val value = control.get(gamepad)
            val deadZone = -control.deadZone..control.deadZone

            when (control.type) {
                Analog.ControlType.CONTINUOUS -> {
                    if (value !in deadZone) {
                        Scheduler.schedule(command(value))
//                        println("Not in deadZone: $value")
                    } else {
                        Scheduler.schedule(command(0.0))
//                        println("In deadZone: $value")
                    }
                }
                Analog.ControlType.REST ->
                    if (value in deadZone && control.previousValue !in deadZone)
                        Scheduler.schedule(command(value))
                Analog.ControlType.CHANGE ->
                    if (value !in deadZone && control.previousValue in deadZone
                        || value in deadZone && control.previousValue !in deadZone
                        || value != control.previousValue)
                    {
                        Scheduler.schedule(command(value))
                    }
            }
        }
    }


    override val beforeRun = Command.continuous("ControlsUpdate") {
        updateDigitalControls(gamepad1, digitalGP1)
        updateDigitalControls(gamepad2, digitalGP2)
        updateAnalogControls(gamepad1, analogGP1)
        updateAnalogControls(gamepad2, analogGP2)
    }

    class Builder {
        val digitalControls = mutableMapOf<Digital, DigitalCommand>()
        val analogControls = mutableMapOf<Analog, AnalogCommand>()

        fun register(control: Digital, command: (value: Boolean) -> Command<*>) {
            digitalControls[control] = command
        }

        fun register(control: Analog, command: (value: Double) -> Command<*>) {
            analogControls[control] = command
        }
    }
}
