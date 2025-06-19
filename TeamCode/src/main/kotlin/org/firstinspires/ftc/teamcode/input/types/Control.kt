package org.firstinspires.ftc.teamcode.input.types

import com.qualcomm.robotcore.hardware.Gamepad

sealed class Control<T> {
    /** Gets the value of the control from a given gamepad. */
    internal abstract val get: (gamepad: Gamepad) -> T

    /** Previously read value of the control. Must be set outside. */
    internal abstract var previousValue: T

    /** Modifier for the control. Mainly used for speed. */
    abstract var modifier: Double
}