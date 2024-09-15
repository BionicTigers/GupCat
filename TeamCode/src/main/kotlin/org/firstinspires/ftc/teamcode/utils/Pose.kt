package org.firstinspires.ftc.teamcode.utils

class Pose(val x: Double, val y: Double, private val rot: Double) {
    constructor(x: Number, y: Number, rot: Number) : this(x.toDouble(), y.toDouble(), rot.toDouble())

    val position: Vector2
        get() = Vector2(x, y)

    val radians: Double
        get() = Math.toRadians(rot)

    val degrees: Double
        get() = rot
}