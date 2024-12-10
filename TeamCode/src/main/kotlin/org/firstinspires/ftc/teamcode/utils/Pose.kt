package org.firstinspires.ftc.teamcode.utils

import kotlin.math.absoluteValue

class Pose(val x: Double, val y: Double, private val rot: Double) {
    constructor(x: Number, y: Number, rot: Number) : this(x.toDouble(), y.toDouble(), rot.toDouble())
    constructor(x: Number, y: Number, rot: Angle) : this(x.toDouble(), y.toDouble(), rot.degrees)

    operator fun compareTo(other: Pose): Int {
        return if (x == other.x && y == other.y && rot == other.rot) {
            0
        } else if (x < other.x || y < other.y || rot < other.rot) {
            1
        } else {
            -1
        }
    }

    fun within(center: Pose, allowance: Pose): Boolean {
        return  center.x.absoluteValue + allowance.x.absoluteValue >= x.absoluteValue && center.x.absoluteValue - allowance.x.absoluteValue <= x.absoluteValue
                && center.y.absoluteValue + allowance.y.absoluteValue >= y.absoluteValue && center.y.absoluteValue - allowance.y.absoluteValue <= y.absoluteValue
                && center.rot.absoluteValue + allowance.rot.absoluteValue >= rot.absoluteValue && center.rot.absoluteValue - allowance.rot.absoluteValue <= rot.absoluteValue
    }

    val position: Vector2
        get() = Vector2(x, y)

    val rotation = Angle.degrees(rot)

    val radians: Double
        get() = Math.toRadians(rot)

    val degrees: Double
        get() = rot

    val absoluteValue: Pose
        get() = Pose(x.absoluteValue, y.absoluteValue, rot.absoluteValue)

    override fun toString(): String {
        return "Pose(x=$x, y=$y, rot=$rot)"
    }

    operator fun plus(other: Pose): Pose {
        return Pose(x + other.x, y + other.y, rot + other.rot)
    }

    operator fun minus(other: Pose): Pose {
        return Pose(x - other.x, y - other.y, rot - other.rot)
    }

    operator fun times(scalar: Double): Pose {
        return Pose(x * scalar, y * scalar, rot * scalar)
    }

    operator fun div(scalar: Double): Pose {
        return Pose(x / scalar, y / scalar, rot / scalar)
    }
}