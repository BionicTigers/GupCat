package org.firstinspires.ftc.teamcode.input.types

import com.qualcomm.robotcore.hardware.Gamepad

class Digital private constructor(
    override val get: (gamepad: Gamepad) -> Boolean = { false }
) : Control<Boolean>() {
    enum class ControlType { HOLD, PRESS, RELEASE }

    var type = ControlType.PRESS

    override var previousValue = false
    override var modifier = 1.0

    fun hold(): Digital {
        type = ControlType.HOLD
        return this
    }

    fun press(): Digital {
        type = ControlType.PRESS
        return this
    }

    fun release(): Digital {
        type = ControlType.RELEASE
        return this
    }

    fun modify(modifier: Double): Digital {
        this.modifier = modifier
        return this
    }

    companion object {
        val A get() = Digital { it.a }
        val B get() = Digital { it.b }
        val X get() = Digital { it.x }
        val Y get() = Digital { it.y }

        val LEFT_BUMPER get() = Digital { it.left_bumper }
        val RIGHT_BUMPER get() = Digital { it.right_bumper }

        val LEFT_STICK_BUTTON get() = Digital { it.left_stick_button }
        val RIGHT_STICK_BUTTON get() = Digital { it.right_stick_button }

        val BACK get() = Digital { it.back }
        val START get() = Digital { it.start }

        val DPAD_UP get() = Digital { it.dpad_up }
        val DPAD_DOWN get() = Digital { it.dpad_down }
        val DPAD_LEFT get() = Digital { it.dpad_left }
        val DPAD_RIGHT get() = Digital { it.dpad_right }
    }
}