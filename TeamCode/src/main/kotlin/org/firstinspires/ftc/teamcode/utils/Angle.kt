package org.firstinspires.ftc.teamcode.utils

import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

class Angle private constructor(val radians: Double) : Comparable<Angle> {
    companion object {
        fun radians(radians: Number) = Angle(radians.toDouble())
        fun degrees(degrees: Number) = Angle(degrees.toDouble() / 180 * PI)
        val ZERO = radians(0)
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

    operator fun times(other: Number): Angle {
        return radians(radians * other.toDouble())
    }

    operator fun div(otherRotation: Angle): Angle {
        return radians(radians / otherRotation.radians)
    }

    operator fun div(other: Number): Angle {
        return radians(radians / other.toDouble())
    }

    operator fun unaryMinus(): Angle {
        return radians(-radians)
    }

    operator fun unaryPlus(): Angle {
        return radians(radians)
    }

    override operator fun compareTo(other: Angle): Int {
        return radians.compareTo(other.radians)
    }

    override fun toString(): String {
        return "${degrees}°"
    }

    val abs: Angle
        get() = degrees(this.degrees.absoluteValue)
    val sin = sin(radians)
    val cos = cos(radians)
    val tan = tan(radians)
}
