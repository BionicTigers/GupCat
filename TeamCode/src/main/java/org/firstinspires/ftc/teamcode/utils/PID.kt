package org.firstinspires.ftc.teamcode.utils

class PID(
    val kP: Double,
    val Ti: Double,
    val Td: Double,
    val cvMin: Double,
    val cvMax: Double,
    val pvMin: Double,
    val pvMax: Double
) {
    private val time

    fun calculate(setPoint: Double, processValue: Double): Double {

    }
}