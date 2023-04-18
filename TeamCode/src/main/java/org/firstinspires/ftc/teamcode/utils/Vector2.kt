package org.firstinspires.ftc.teamcode.utils;

import kotlin.math.sqrt

data class Vector2(val x: Double, val y: Double) {
    constructor(): this(0.0, 0.0)

    fun magnitude(): Double {
        return sqrt(x * x + y * y)
    }
}