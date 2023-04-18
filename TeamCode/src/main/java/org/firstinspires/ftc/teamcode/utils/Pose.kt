package org.firstinspires.ftc.teamcode.utils

data class Pose(val x: Double, val y: Double, val rotation: Double) {

    constructor(): this(0.0, 0.0, 0.0)
    constructor(v2: Vector2, rotation: Double): this(v2.x, v2.y, rotation)

    fun extractPosition(): Vector2 {
        return Vector2(x, y)
    }
}