package org.firstinspires.ftc.teamcode.utils

import kotlin.math.pow

class Distance private constructor(val mm: Double) {
    companion object {
        fun mm(value: Number) = Distance(value.toDouble())
        fun cm(value: Number) = Distance(value.toDouble() * 10)
        fun m(value: Number) = Distance(value.toDouble() * 1000)
        fun inch(value: Number) = Distance(value.toDouble() * 25.4)
        fun ft(value: Number) = Distance(value.toDouble() * 304.8)
    }

    val cm: Double
        get() = mm / 10

    val m: Double
        get() = mm / 1000

    val inch: Double
        get() = mm / 25.4

    val ft: Double
        get() = mm / 304.8

    fun pow(exponent: Double) = Distance(mm.pow(exponent))
    fun sqrt() = Distance(kotlin.math.sqrt(mm))

    operator fun plus(distance: Distance) = Distance(mm + distance.mm)
    operator fun minus(distance: Distance) = Distance(mm - distance.mm)
    operator fun times(distance: Distance) = Distance(mm * distance.mm)
    operator fun times(scalar: Double) = Distance(mm * scalar)
    operator fun div(distance: Distance) = Distance(mm / distance.mm)
    operator fun div(scalar: Double) = Distance(mm / scalar)
    operator fun unaryMinus() = Distance(-mm)
    operator fun compareTo(distance: Distance) = mm.compareTo(distance.mm)
    operator fun compareTo(scalar: Number) = mm.compareTo(scalar.toDouble())
}