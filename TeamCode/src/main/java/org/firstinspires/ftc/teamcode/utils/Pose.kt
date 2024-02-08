package org.firstinspires.ftc.teamcode.utils

import kotlin.math.absoluteValue
import kotlin.math.sqrt

/**
 * Combines a Vector2 and a Rotation variable
 * Used for robot position and rotation
 */
data class Pose(val x: Double, val y: Double, val rotation: Double) {
    constructor(): this(0.0, 0.0, 0.0)
    constructor(v2: Vector2, rotation: Double): this(v2.x, v2.y, rotation)

    val radians: Double
        get() {
            return Math.toRadians(rotation)
        }

    /**
     * Convert the pose x and y into a Vector2
     */
    fun extractPosition(): Vector2 {
        return Vector2(x, y)
    }

//    fun getRotationError(other: Double): Double {
//        // (810 / 360) - floor(810 / 360) = .25
//        // (90 / 360) - (810 / 360) = .25 - 1.25 = -1
////        val normalizedRotation = (this.rotation / 360) - (this.rotation / 360).toInt()
//        return ((other / 360) - (this.rotation / 360)) * 360
//    }

    fun abs(): Pose {
        return Pose(this.x.absoluteValue, this.y.absoluteValue, this.rotation.absoluteValue)
    }

    operator fun compareTo(other: Pose): Int {
        val thisDistance = sqrt(x * x + y * y)
        val otherDistance = sqrt(other.x * other.x + other.y * other.y)

        val thisCombined = thisDistance + rotation
        val otherCombined = otherDistance + other.rotation

        return thisCombined.compareTo(otherCombined)
    }

    operator fun plus(other: Pose): Pose {
        return Pose(this.x + other.x, this.y + other.y, this.rotation + other.rotation)
    }

    operator fun minus(other: Pose): Pose {
        return Pose(this.x - other.x, this.y - other.y, this.rotation - other.rotation)
    }

    operator fun div(other: Pose): Pose {
        return Pose(this.x / other.x, this.y / other.y, this.rotation / other.rotation)
    }

    operator fun div(other: Double): Pose {
        return Pose(this.x / other, this.y / other, this.rotation / other)
    }

    operator fun times(other: Pose): Pose {
        return Pose(this.x * other.x, this.y * other.y, this.rotation * other.rotation)
    }

    operator fun unaryMinus(): Pose {
        return Pose(-this.x, -this.y, -this.rotation)
    }
}