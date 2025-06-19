package org.firstinspires.ftc.teamcode.input.profiles

import org.firstinspires.ftc.teamcode.input.types.Control
import org.firstinspires.ftc.teamcode.input.types.Digital
import org.firstinspires.ftc.teamcode.input.Gamepads
import org.firstinspires.ftc.teamcode.input.Profile
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.utils.Angle

open class Default : Profile {
    override val pivot = object : Pivot.Schema {
        override val desiredGamepad = Gamepads.GAMEPAD_2
        override val rate = Angle.degrees(72.5)
        override val up = Digital.RIGHT_BUMPER.hold()
        override val down = Digital.LEFT_BUMPER.hold()
        override val min = null
        override val max = null
    }

    override val slides = object : Slides.Schema {
        override val desiredGamepad = Gamepads.GAMEPAD_2
        override val rate = 37600
        override val raise: Control<*> = Digital.DPAD_UP.hold()
        override val lower: Control<*> = Digital.DPAD_DOWN.hold()
        override val min: Digital? = null
        override val max: Digital? = null
    }

    override val arm = object : Arm.Schema {
        override val desiredGamepad = Gamepads.GAMEPAD_2
        override val toggleUpDown = Digital.B.hold()
        override val up = null
        override val middle = Digital.Y.hold()
        override val down = null
    }

    override val claw = object : Claw.Schema {
        override val desiredGamepad = Gamepads.GAMEPAD_2
        override val open = null
        override val close = null
        override val toggle = Digital.A.hold()
    }
}