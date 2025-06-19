package io.github.bionictigers.axiom.web

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max

object WebData {
    /**
     * Drivetrain powers, in the order { FL, FR, BL, BR }
     * 
     * Values are normalized to be between -1 and 1
     * 
     * If null, the drivetrain will not be accessible in the web interface
     */
    var drivetrain: List<Double>? = null
        set(value) {
            if (value != null) {
                if (value.size != 4) error("Drivetrain Powers must be { FL, FR, BL, BR }")
                val max = max(value.maxOf { abs(it) }, 1.0)
                field = value.map { it / max }
            } else {
                field = null
            }
        }

    /**
     * Set the drivetrain powers
     * 
     * @param fl The power of the front left wheel
     * @param fr The power of the front right wheel
     * @param bl The power of the back left wheel
     * @param br The power of the back right wheel
     */
    fun setDrivetrain(fl: Double, fr: Double, bl: Double, br: Double) {
        drivetrain = listOf(fl, fr, bl, br)
    }

    /**
     * The x position of the robot in mm from the blue bucket corner
     */
    var x: Double? = null

    /**
     * The y position of the robot in mm from the blue bucket corner
     */
    var y: Double? = null

    /**
     * The rotation of the robot in radians
     *
     * If the robot is facing the red side, that is 0
     * 
     * Values are normalized to be between 0 and 2*PI
     */
    var rotation: Double? = null
        set(value) {
            field = if (value != null) {
                value % (2 * PI)
            } else {
                null
            }
        }
}