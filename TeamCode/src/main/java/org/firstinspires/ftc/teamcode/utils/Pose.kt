package org.firstinspires.ftc.teamcode.utils

data class Pose(val x: Float, val y: Float, val rotation: Float) {

    constructor(): this(0f, 0f, 0f)
    constructor(v2: Vector2, rotation: Float): this(v2.x, v2.y, rotation)

    fun extractPosition(): Vector2 {
        return Vector2(x, y)
    }
}