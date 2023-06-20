package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.ElapsedTime

class PID(
    val kP: Double,
    val tI: Double,
    val tD: Double,
    private val pvMin: Double,
    private val pvMax: Double,
    private val cvMin: Double,
    private val cvMax: Double)
{
    private val sampleTime = 20 //In Milliseconds

    private val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

    //Expose P, I, D, E, CV variables to outside the class
    var p: Double = 0.0
        internal set
    var i: Double = 0.0
        internal set
    var d: Double = 0.0
        internal set
    var previousError: Double = 0.0
        internal set
    var cv: Double = 0.0
        internal set

    constructor(terms: PIDTerms,
                pvMin: Double,
                pvMax: Double,
                cvMin: Double,
                cvMax: Double): this(terms.kP, terms.tI, terms.tD, pvMin, pvMax, cvMin, cvMax)

    fun calculate(setPoint: Double, processValue: Double): Double {
        val dt = elapsedTime.seconds()

        if (dt < sampleTime / 1000) {
            val error = (setPoint - processValue) / (pvMax - pvMin)

            p = kP * error
            i += if (cv > cvMin && cv < cvMax && tI != 0.0) 1 / 60 * tI * error else 0.0
            d = kP * tD / 60 * (previousError - error)

            cv = p + kP * i + d

            previousError = error
            elapsedTime.reset()
        }

        return cv
    }
}