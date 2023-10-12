package org.firstinspires.ftc.teamcode.utils

import kotlin.math.pow

private data class TimeSlices(val t1: Double,
                              val t2: Double,
                              val t3: Double,
                              val t4: Double,
                              val t5: Double,
                              val t6: Double,
                              val t7: Double)

/**
 * Generate a motion profile based off certain parameters
 *
 * @param jerk mm/s^3
 * @param maxAcceleration mm/s^2
 * @param maxVelocity mm/s
 */
fun generateMotionProfile(start: Double,
                          target: Double,
                          jerk: Double,
                          maxAcceleration: Double,
                          maxVelocity: Double)
{
    val t1 = maxAcceleration / jerk
    val v1 = (t1 * maxAcceleration) / 2
    val v2 = maxVelocity - v1
    val t2 = t1 + ((maxVelocity - maxAcceleration.pow(2) / jerk) / maxAcceleration)
    val p1 = jerk * (t1.pow(3)/6)
    val p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2)/2
    val p3 = p2 + v2 * (t1 - t2) + maxAcceleration * (t1 - t2).pow(2)/2 - jerk * (t1 - t2).pow(3)/6



    val slices = TimeSlices(t1, t2, t1, 0.0, t1, t2, t1)
}