package org.firstinspires.ftc.teamcode.utils.input

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.utils.Vector2

fun Gamepad.into() = GamepadEx(this)

class GamepadEx(private val gamepad: Gamepad) {
    //Gamepad Buttons
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

    //Store Buttons
    private val boolButtons: ArrayList<BoolButton> = ArrayList()

    //Store Joysticks
    private val leftJoystick: Joystick = Joystick()
    private val rightJoystick: Joystick = Joystick()

    //Store Trigger
    private val leftTrigger: Trigger = Trigger()
    private val rightTrigger: Trigger = Trigger()

    //Assign BoolButtons to array
    init {
        for (button in Buttons.values()) {
            boolButtons.add(button.ordinal, BoolButton())
        }
    }

    //Get a BoolButton
    fun getButton(button: Buttons): BoolButton {
        return boolButtons[button.ordinal]
    }

    //Get a Joystick
    fun getJoystick(joystick: Joysticks): Joystick {
        return if (joystick == Joysticks.LEFT_JOYSTICK) {
            leftJoystick
        } else {
            rightJoystick
        }
    }

    fun getJoysticks(): Pair<Joystick, Joystick> {
        return Pair(leftJoystick, rightJoystick)
    }

    fun getTrigger(trigger: Triggers): Trigger {
        return if (trigger == Triggers.LEFT_TRIGGER) {
            leftTrigger
        } else {
            rightTrigger
        }
    }

    //Update every button and joystick
    fun update() {
        boolButtons.forEachIndexed { index, button ->
            //Find the name of the field
            //Call update with the value of the field using reflection
            val name = Buttons.entries[index].name.lowercase()
            button.update(gamepad.javaClass.getDeclaredField(name).getBoolean(gamepad))
        }

        leftJoystick.update(
            Vector2(
                gamepad.left_stick_x.toDouble(),
                gamepad.left_stick_y.toDouble()
            )
        )
        rightJoystick.update(
            Vector2(
                gamepad.right_stick_x.toDouble(),
                gamepad.right_stick_y.toDouble()
            )
        )

        leftTrigger.update(gamepad.left_trigger.toDouble())
        rightTrigger.update(gamepad.right_trigger.toDouble())
    }
}