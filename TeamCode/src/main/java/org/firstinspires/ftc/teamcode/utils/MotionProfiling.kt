package org.firstinspires.ftc.teamcode.utils

import kotlin.math.pow
import kotlin.math.sqrt

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
                          maxVelocity: Double,
                          points: Int = 500)
{
    val va = maxAcceleration.pow(2) / jerk
    val sa = 2 * maxAcceleration.pow(3) / jerk.pow(2)
    val sv = if (maxVelocity * jerk < maxAcceleration.pow(2)) 2 * maxVelocity * sqrt(maxVelocity / jerk) else maxVelocity * (maxVelocity / maxAcceleration + maxAcceleration / jerk)

    if ((va > maxVelocity && sa < target) || (va > maxVelocity && sa > target && sv < target)) {
        //A & C.1
    } else if ((va < maxVelocity && sa > target) || (va > maxVelocity && sa > target && sv > target)) {
        //B & C.2
    } else if (va < maxVelocity && sa < target && sv > target) {
        //D.1
    } else {
        val t1 = maxAcceleration / jerk
        val v1 = (t1 * maxAcceleration) / 2
        val v2 = maxVelocity - v1
        val t2 = t1 + ((maxVelocity - maxAcceleration.pow(2) / jerk) / maxAcceleration)
        val p1 = jerk * (t1.pow(3)/6)
        val p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2)/2
        val p3 = p2 + v2 * (t1 - t2) + maxAcceleration * (t1 - t2).pow(2)/2 - jerk * (t1 - t2).pow(3)/6
        val p4 = p3 + target - 2 * p3
        val p5 = p4 + (p3 - p2)
        val p6 = p5 + (p2 - p1)
        val t3 = t1 + t2
        val t4 = t3 + (target - 2 * p3) / maxVelocity
        val t5 = t4 + t1
        val t6 = t5 + (t2-t1)
        val t7 = t6 + t1

    }
}