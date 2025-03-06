package org.firstinspires.ftc.teamcode.utils

import kotlin.math.sqrt

/**
 * A 2D vector.
 * @param x The x component.
 * @param y The y component.
 */
data class Vector2(var x: Double, var y: Double) {
    constructor(x: Number, y: Number) : this(x.toDouble(), y.toDouble())
    constructor() : this(0.0, 0.0)

    fun magnitude(): Double {
        return sqrt(x * x + y * y)
    }

    fun normalize(): Vector2 {
        return this/this.magnitude()
    }

    operator fun div(num: Number): Vector2 {
        return Vector2(this.x / num.toDouble(), this.y / num.toDouble())
    }

    operator fun div(other: Vector2): Vector2 {
        return Vector2(this.x / other.x, this.y / other.y)
    }

    operator fun minus(num: Number): Vector2 {
        return Vector2(this.x - num.toDouble(), this.y - num.toDouble())
    }

    operator fun minus(other: Vector2): Vector2 {
        return Vector2(this.x - other.x, this.y - other.y)
    }

    operator fun times(num: Number): Vector2 {
        return Vector2(this.x * num.toDouble(), this.y * num.toDouble())
    }
}
