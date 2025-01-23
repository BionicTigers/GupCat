package org.firstinspires.ftc.teamcode.utils

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

class Angle private constructor(val radians: Double) {
    companion object {
        fun radians(radians: Number) = Angle(radians.toDouble())
        fun degrees(degrees: Number) = Angle(degrees.toDouble() / 180 * PI)
    }

    val degrees: Double
        get() = radians * 180 / PI

    operator fun plus(otherRotation: Angle): Angle {
        return radians(radians + otherRotation.radians)
    }

    operator fun minus(otherRotation: Angle): Angle {
        return radians(radians - otherRotation.radians)
    }

    operator fun times(otherRotation: Angle): Angle {
        return radians(radians * otherRotation.radians)
    }

    operator fun div(otherRotation: Angle): Angle {
        return radians(radians / otherRotation.radians)
    }

    operator fun div(scalar: Number): Angle {
        return radians(radians / scalar.toDouble())
    }

    operator fun unaryMinus(): Angle {
        return radians(-radians)
    }

    operator fun unaryPlus(): Angle {
        return radians(radians)
    }

    operator fun compareTo(otherRotation: Angle): Int {
        return radians.compareTo(otherRotation.radians)
    }

    override fun toString(): String {
        return "Degrees: $degrees, Radians: $radians"
    }

    val sin = sin(radians)
    val cos = cos(radians)
    val tan = tan(radians)
}
