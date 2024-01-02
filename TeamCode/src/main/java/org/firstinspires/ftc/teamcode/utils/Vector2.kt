package org.firstinspires.ftc.teamcode.utils;

import kotlin.math.sqrt

data class Vector2(val x: Double, val y: Double) {
    constructor(): this(0.0, 0.0)

    fun magnitude(): Double {
        return sqrt(x * x + y * y)
    }

    fun normalize(): Vector2 {
        return this/this.magnitude()
    }

    operator fun div(num: Double): Vector2 {
        return Vector2(this.x / num, this.y / num)
    }

    operator fun minus(other: Vector2): Vector2 {
        return Vector2(this.x - other.x, this.y - other.y)
    }

    operator fun times(other: Vector2): Vector2 {
        return Vector2(this.x * other.x, this.y * other.y)
    }
}