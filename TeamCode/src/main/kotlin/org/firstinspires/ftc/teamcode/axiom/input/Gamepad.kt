package org.firstinspires.ftc.teamcode.axiom.input

import com.qualcomm.robotcore.hardware.Gamepad as FTCGamepad
import io.github.bionictigers.input.Joystick
import io.github.bionictigers.input.Trigger
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler

interface GamepadState : CommandState {
    //Store Buttons
    val boolButtons: Map<Gamepad.Buttons, BooleanButton>

    //Store Joysticks
    val leftJoystick: Joystick
    val rightJoystick: Joystick

    //Store Trigger
    val leftTrigger: Trigger
    val rightTrigger: Trigger
}

class Gamepad(private val ftcGamepad: FTCGamepad, system: GamepadSystem) {
    enum class Buttons {
        DPAD_UP, DPAD_DOWN, DPAD_RIGHT, DPAD_LEFT, A, B, X, Y, START,
        BACK, LEFT_BUMPER, RIGHT_BUMPER, LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON, GUIDE
    }

    enum class Joysticks {
        LEFT_JOYSTICK, RIGHT_JOYSTICK
    }

    enum class Triggers {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    internal val command = Command<GamepadState>(object : GamepadState, CommandState by CommandState.default("Gamepad") {
        override val boolButtons: Map<Buttons, BooleanButton> = Buttons.entries.associateWith { BooleanButton(false) }
        override val leftJoystick: Joystick = Joystick(Vector2(0.0, 0.0))
        override val rightJoystick: Joystick = Joystick(Vector2(0.0, 0.0))
        override val leftTrigger: Trigger = Trigger(0.0)
        override val rightTrigger: Trigger = Trigger(0.0)
    })
        .setAction {
            it.boolButtons.forEach { (index, button) ->
                val field = FTCGamepad::class.java.getDeclaredField(index.name.lowercase())
                field.isAccessible = true
                val value = field.getBoolean(ftcGamepad)
                button.update(value).forEach {
                    it(value)
                }
            }

            val leftJoystickValue = Vector2(ftcGamepad.left_stick_x.toDouble(), ftcGamepad.left_stick_y.toDouble())
            it.leftJoystick.update(leftJoystickValue).forEach { it(leftJoystickValue) }

            val rightJoystickValue = Vector2(ftcGamepad.right_stick_x.toDouble(), ftcGamepad.right_stick_y.toDouble())
            it.rightJoystick.update(rightJoystickValue).forEach { it(rightJoystickValue) }

            val leftTriggerValue = ftcGamepad.left_trigger.toDouble()
            it.leftTrigger.update(leftTriggerValue).forEach { it(leftTriggerValue) }

            val rightTriggerValue = ftcGamepad.right_trigger.toDouble()
            it.rightTrigger.update(rightTriggerValue).forEach { it(rightTriggerValue) }

            false
        }

    val leftJoystick get() = command.state.leftJoystick
    val rightJoystick get() = command.state.rightJoystick

    val leftTrigger get() = command.state.leftTrigger
    val rightTrigger get() = command.state.rightTrigger

    fun getBooleanButton(button: Buttons): BooleanButton {
        return command.state.boolButtons[button]!!
    }
}