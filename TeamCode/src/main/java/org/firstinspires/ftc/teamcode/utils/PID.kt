package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.ElapsedTime

class PID(
    private val kP: Double,
    private val Ti: Double, // in minutes
    private val Td: Double, // in minutes
    private val cvMin: Double,
    private val cvMax: Double,
    private val pvMin: Double,
    private val pvMax: Double
) {
    private val sampleTime = 20 // in ms

    private var pv1: Double = 0.0
    private var pv2: Double = 0.0

    private var e1: Double = 0.0

    private var cv: Double = 0.0

    private val previousCall = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    fun calculate(setPoint: Double, processValue: Double): Double {
        val dt = previousCall.milliseconds() / 1000

        if (dt > sampleTime / 1000) {
            val scaledPV = processValue / (pvMax - pvMin)
            val e = (setPoint - processValue) / (pvMax - pvMin)

            val p = e - e1

            val i = if (Ti == 0.0) 0.0 else e * dt / 60 * Ti // Can't divide by 0 so we set to 0

            val d = 60 * Td * (scaledPV - 2 * pv1 + pv2) / dt

            //Clamp CV to the min and the max
            cv = (cv + kP * (p + i + d)).coerceIn(cvMin, cvMax)

            //Set Futures
            e1 = e

            pv2 = pv1
            pv1 = scaledPV

            //Reset Elapsed Time
            previousCall.reset()
        }

        return cv
    }
}