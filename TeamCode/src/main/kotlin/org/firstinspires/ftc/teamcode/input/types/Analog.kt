package org.firstinspires.ftc.teamcode.input.types

import com.qualcomm.robotcore.hardware.Gamepad

class Analog private constructor(
    override val get: (gamepad: Gamepad) -> Double,
) : Control<Double>() {
    enum class ControlType { REST, CHANGE, CONTINUOUS }

    var type = ControlType.CONTINUOUS
    var deadZone: Double = 0.0

    override var previousValue = 0.0
    override var modifier = 1.0

    fun rest(): Analog {
        type = ControlType.REST
        return this
    }

    fun change(): Analog {
        type = ControlType.CHANGE
        return this
    }

    fun continuous(): Analog {
        type = ControlType.CONTINUOUS
        return this
    }

    fun deadZone(deadZone: Double): Analog {
        this.deadZone = deadZone
        return this
    }

    fun modify(modifier: Double): Analog {
        this.modifier = modifier
        return this
    }

    companion object {
        val LEFT_STICK_X get() = Analog { it.left_stick_x.toDouble() }
        val LEFT_STICK_Y get() = Analog { it.left_stick_y.toDouble() }
        val RIGHT_STICK_X get() = Analog { it.right_stick_x.toDouble() }
        val RIGHT_STICK_Y get() = Analog { it.right_stick_y.toDouble() }

        val LEFT_TRIGGER get() = Analog { it.left_trigger.toDouble() }
        val RIGHT_TRIGGER get() = Analog { it.right_trigger.toDouble() }
    }
}