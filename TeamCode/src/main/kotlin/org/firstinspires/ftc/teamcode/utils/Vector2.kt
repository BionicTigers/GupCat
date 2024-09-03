package org.firstinspires.ftc.teamcode.utils

import kotlin.math.sqrt

/**
 * A 2D vector.
 * @param x The x component.
 * @param y The y component.
 *
 * @param T The type of the components.
 */
data class Vector2<T: Number>(val x: Double = 0.0, val y: Double = 0.0) {
    constructor(x: T, y: T) : this(x.toDouble(), y.toDouble())

    fun magnitude(): Double {
        return sqrt(x * x + y * y)
    }

    fun normalize(): Vector2<Double> {
        return this/this.magnitude()
    }

    operator fun <V : Number> div(num: V): Vector2<Double> {
        return Vector2(this.x / num.toDouble(), this.y / num.toDouble())
    }

    operator fun <V : Number> minus(num: V): Vector2<Double> {
        return Vector2(this.x - num.toDouble(), this.y - num.toDouble())
    }

    operator fun <V : Number> times(num: V): Vector2<Double> {
        return Vector2(this.x * num.toDouble(), this.y * num.toDouble())
    }
}