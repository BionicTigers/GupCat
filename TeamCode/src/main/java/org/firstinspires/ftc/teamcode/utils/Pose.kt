package org.firstinspires.ftc.teamcode.utils

import kotlin.math.abs

/**
 * Combines a Vector2 and a Rotation variable
 * Used for robot position and rotation
 */
data class Pose(val x: Double, val y: Double, val rotation: Double) {

    constructor(): this(0.0, 0.0, 0.0)
    constructor(v2: Vector2, rotation: Double): this(v2.x, v2.y, rotation)

    /**
     * Convert the pose x and y into a Vector2
     */
    fun extractPosition(): Vector2 {
        return Vector2(x, y)
    }

    //TODO: Change this to a operator override
    fun compare(x: Double, y: Double, rotation: Double): Boolean {
        return abs(this.x) <= x && abs(this.y) <= y && abs(this.rotation) <= rotation
    }

    operator fun minus(other: Pose): Pose {
        return Pose(this.x - other.x, this.y - other.y, this.rotation - other.rotation)
    }
}