package org.firstinspires.ftc.teamcode.utils

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

    fun compare(x: Double, y: Double, rotation: Double): Boolean {
        return this.x <= x && this.y <= y && this.rotation <= rotation
    }

    operator fun minus(other: Pose): Pose {
        return Pose(this.x - other.x, this.y - other.y, this.rotation - other.rotation)
    }
}