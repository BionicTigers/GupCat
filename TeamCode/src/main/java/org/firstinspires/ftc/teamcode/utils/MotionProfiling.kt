package org.firstinspires.ftc.teamcode.utils

import kotlin.math.pow

private data class timeSlices(val t1: Double,
                              val t2: Double,
                              val t3: Double,
                              val t4: Double,
                              val t5: Double,
                              val t6: Double,
                              val t7: Double,
                              val t8: Double)

/**
 * Generate a motion profile based off certain parameters
 *
 * @param jerk mm/s^3
 * @param maxAcceleration mm/s^2
 * @param maxVelocity mm/s
 */
fun generateMotionProfile(target: Double,
                          jerk: Double,
                          maxAcceleration: Double,
                          maxVelocity: Double)
{
    val t1 = (target / (2 * jerk)).pow(1/3)
    val t2 = t1
    val t3 = 2 * t1
    val t4 = t3
    val t5 = 3 * t1
    val t6 = t1
    val t7 = 4 * t1
}