package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.util.ElapsedTime
import io.github.bionictigers.axiom.commands.Display
import io.github.bionictigers.axiom.commands.Value

/**
 * Class to control errors
 * Takes in either PIDTerms (Used for realtime editing) or kP, tI, tD
 * Takes in pvMin and pvMax which is the minimum and maximum that the process value can reach
 * Also takes in cvMin and cvMax which is the maximum output, usually -1 to 1
 *
 * @param kP Proportional constant
 * @param tI Integral time
 * @param tD Derivative time
 * @param pvMin Minimum process value
 * @param pvMax Maximum process value
 * @param cvMin Minimum control variable
 * @param cvMax Maximum control variable
 */
class PID(
    var kP: Double,
    var tI: Double,
    var tD: Double,
    var pvMin: Double,
    var pvMax: Double,
    var cvMin: Double,
    var cvMax: Double,
    private val sampleTime: Int = 20
) /*: Display()*/ {
    //Time between cycles, in ms

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
                cvMax: Double,
                sampleTime: Int = 100
        ): this(terms.kP, terms.tI, terms.tD, pvMin, pvMax, cvMin, cvMax, sampleTime)

    /**
     * Calculate the control variable (Output)
     */
    fun calculate(setPoint: Double, processValue: Double): Double {
        val dt = elapsedTime.seconds()

        //Check if pid should calculate again
        if (dt > sampleTime / 1000.0) {
            //Error in terms of processValue (percentage)
            val error = ((setPoint - processValue) / (pvMax - pvMin)).let { if (it.isNaN() || it.isInfinite()) 0.0 else it }

            p = kP * error
            i += if (cv > cvMin && cv < cvMax && tI != 0.0) 1.0 / tI * error else 0.0
            d = kP * tD / 60.0 * (previousError - error)

            //Multiply I by kP last to allow for real time editing
            cv = p + kP * i + d

            //Prepare for next call
            previousError = error
            elapsedTime.reset()
        }

        return cv.coerceIn(cvMin, cvMax)
    }

    /**
     * Resets the integral and cv
     */
    fun reset() {
        i = 0.0
        cv = 0.0
    }

    fun log() {
        println("P: $p, I: $i, D: $d, E: $previousError")
    }

//    override fun serialize(): Map<String, Value> {
//        return mapOf("cv" to Value(cv, true))
//    }
}