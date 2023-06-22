package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Class to control errors
 * Takes in either PIDTerms (Used for realtime editing) or kP, tI, tD
 * Takes in pvMin and pvMax which is the minimum and maximum that the process value can reach
 * Also takes in cvMin and cvMax which is the maximum output, usually -1 to 1
 */
class PID(
    val kP: Double,
    val tI: Double,
    val tD: Double,
    private val pvMin: Double,
    private val pvMax: Double,
    private val cvMin: Double,
    private val cvMax: Double)
{
    //Time between cycles, in ms
    private val sampleTime = 20

    private val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

    //Expose P, I, D, E, CV variables to outside the class for logging
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

    /**
     * Uses the PIDTerms class instead of kP, tI, tD
     */
    constructor(terms: PIDTerms,
                pvMin: Double,
                pvMax: Double,
                cvMin: Double,
                cvMax: Double): this(terms.kP, terms.tI, terms.tD, pvMin, pvMax, cvMin, cvMax)

    /**
     * Calculate the control variable (Output)
     */
    fun calculate(setPoint: Double, processValue: Double): Double {
        val dt = elapsedTime.seconds()

        //Check if pid should calculate again
        if (dt > sampleTime / 1000) {
            //Error in terms of processValue (percentage)
            val error = (setPoint - processValue) / (pvMax - pvMin)

            p = kP * error
            i += if (cv > cvMin && cv < cvMax && tI != 0.0) 1 / 60 * tI * error else 0.0
            d = kP * tD / 60 * (previousError - error)

            //Multiply I by kP last to allow for real time editing
            cv = p + kP * i + d

            //Prepare for next call
            previousError = error
            elapsedTime.reset()
        }

        return cv
    }

    /**
     * Resets the integral and cv
     */
    fun reset() {
        i = 0.0
        cv = 0.0
    }

    fun log() {
        println("P: $p, I: $i, D: $d, E: $previousError, CV: $cv")
    }
}