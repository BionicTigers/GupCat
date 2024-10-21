package org.firstinspires.ftc.teamcode.utils

class Pose(val x: Double, val y: Double, private val rot: Double) {
    constructor(x: Number, y: Number, rot: Number) : this(x.toDouble(), y.toDouble(), rot.toDouble())

    val position: Vector2
        get() = Vector2(x, y)

    val radians: Double
        get() = Math.toRadians(rot)

    val degrees: Double
        get() = rot

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