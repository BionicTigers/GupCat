package org.firstinspires.ftc.teamcode.utils

import kotlin.math.sqrt

/**
 * A 2D vector.
 * @param x The x component.
 * @param y The y component.
 */
data class Vector2(val x: Double, val y: Double) {
    constructor(x: Number, y: Number) : this(x.toDouble(), y.toDouble())
    constructor() : this(0.0, 0.0)

    fun magnitude(): Double {
        return sqrt(x * x + y * y)
    }

    fun normalize(): Vector2 {
        return this/this.magnitude()
    }

    operator fun <V : Number> div(num: V): Vector2 {
        return Vector2(this.x / num.toDouble(), this.y / num.toDouble())
    }

    operator fun <V : Number> minus(num: V): Vector2 {
        return Vector2(this.x - num.toDouble(), this.y - num.toDouble())
    }
    operator fun minus(otherVector2: Vector2): Vector2 {
        return Vector2(this.x - otherVector2.x, this.y - otherVector2.y)
    }

    operator fun <V : Number> times(num: V): Vector2 {
        return Vector2(this.x * num.toDouble(), this.y * num.toDouble())
    }
}
