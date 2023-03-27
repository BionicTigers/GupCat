package org.firstinspires.ftc.teamcode.utils;

import kotlin.math.sqrt

data class Vector2(val x: Float, val y: Float) {
    constructor(): this(0f, 0f)

    fun magnitude(): Float {
        return sqrt(x * x + y * y)
    }
}