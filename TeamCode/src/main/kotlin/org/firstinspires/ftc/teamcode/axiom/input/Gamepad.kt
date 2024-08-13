package org.firstinspires.ftc.teamcode.axiom.input

import com.qualcomm.robotcore.hardware.Gamepad as FTCGamepad
import io.github.bionictigers.commands.System
import io.github.bionictigers.input.BooleanButton
import io.github.bionictigers.input.Joystick
import io.github.bionictigers.input.Trigger
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler

interface GamepadState : CommandState {
    //Store Buttons
    val boolButtons: List<BooleanButton>

    //Store Joysticks
    val leftJoystick: Joystick
    val rightJoystick: Joystick

    //Store Trigger
    val leftTrigger: Trigger
    val rightTrigger: Trigger
}

class Gamepad(val ftcGamepad: FTCGamepad, val system: GamepadSystem) {
    enum class Buttons {
        DPAD_UP, DPAD_DOWN, DPAD_RIGHT, DPAD_LEFT, A, B, X, Y, START,
        BACK, LEFT_BUMPER, RIGHT_BUMPER, LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON, GUIDE
    }

    //Gamepad Joysticks
    enum class Joysticks {
        LEFT_JOYSTICK, RIGHT_JOYSTICK
    }

    enum class Triggers {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    init {
        val state = object : GamepadState, CommandState by CommandState.default("Gamepad") {
            override val boolButtons: List<BooleanButton> = Buttons.entries.map { BooleanButton(false) }
            override val leftJoystick: Joystick = Joystick(Vector2(0.0, 0.0))
            override val rightJoystick: Joystick = Joystick(Vector2(0.0, 0.0))
            override val leftTrigger: Trigger = Trigger(0.0)
            override val rightTrigger: Trigger = Trigger(0.0)
        }

        val command = Command<GamepadState>(state)
            .dependsOn(system)
            .setAction {
                it.boolButtons.forEachIndexed { index, button ->
                    val name = Buttons.entries[index].name.lowercase()
                    button.update(FTCGamepad::class.java.getDeclaredField(name).getBoolean(ftcGamepad))
                }

                return@setAction true
            }

        Scheduler.add()
    }
}