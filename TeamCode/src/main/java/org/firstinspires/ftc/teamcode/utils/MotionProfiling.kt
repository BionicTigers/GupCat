package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

data class MotionResult(
    val acceleration: List<Double>,
    val velocity: List<Double>,
    val position: List<Double>,
    val time: List<Double>,
    val deltaTime: Double,
    private val target: Double
) {
    fun getPosition(time: Time): Double {
        return position.getOrElse((time.seconds() / deltaTime).toInt()) {position.last()}
    }

    fun getAcceleration(time: Time): Double {
        return acceleration.getOrElse((time.seconds() / deltaTime).toInt()) {acceleration.last()}
    }

    fun getVelocity(time: Time): Double {
        return velocity.getOrElse((time.seconds() / deltaTime).toInt()) {velocity.last()}
    }
}

/**
 * Generate a motion profile based off certain parameters
 *
 * @param jerk mm/s^3
 * @param maxAcceleration mm/s^2
 * @param maxVelocity mm/s
 */
/**
 * Generate a motion profile based off certain parameters
 *
 * @param jerk mm/s^3
 * @param maxAcceleration mm/s^2
 * @param maxVelocity mm/s
 */
fun generateMotionProfile(
    start: Double,
    final: Double,
    jerk: Double,
    maxAcceleration: Double,
    maxVelocity: Double,
    points: Int = 500
): MotionResult {
    val va = maxAcceleration.pow(2) / jerk
    val sa = 2.0 * maxAcceleration.pow(3) / jerk.pow(2)
    val sv =
        if (maxVelocity * jerk < maxAcceleration.pow(2)) 2 * maxVelocity * sqrt(maxVelocity / jerk) else maxVelocity * (maxVelocity / maxAcceleration + maxAcceleration / jerk)

    val t1: Double
    val t2: Double
    val t3: Double
    val t4: Double
    val t5: Double
    val t6: Double
    val t7: Double

    val p1: Double
    val p2: Double
    val p3: Double
    val p4: Double
    val p5: Double
    val p6: Double

    val v1: Double
    val v2: Double
    val v3: Double
    val v4: Double
    val v5: Double
    val v6: Double

    val a1: Double
    val a2: Double
    val a3: Double
    val a4: Double
    val a5: Double
    val a6: Double

    val target = abs(final - start)
    if (target <= 0) return MotionResult(listOf(0.0), listOf(0.0), listOf(start), listOf(0.0), 1.0, start)

    if ((va > maxVelocity && sa < target) || (va > maxVelocity && sa > target && sv < target)) {
        //A & C.1
        println("A & C.1")
        t1 = (maxVelocity / jerk).pow(.5)
        p1 = (jerk * t1.pow(3)) / 6.0
        v1 = (jerk * t1.pow(2)) / 2.0
        t2 = t1
        v2 = v1
        p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2) / 2
        t3 = 2 * t1
        v3 = maxVelocity
        p3 = maxVelocity * (maxVelocity / jerk).pow(.5)
        t4 = t3 + (target - 2 * p3) / maxVelocity
        v4 = maxVelocity
        p4 = p3 + target - 2 * p3
        t5 = t4 + t1
        v5 = v4 - v1
        p5 = target - p1
        t6 = t5
        v6 = v5
        p6 = p5
        t7 = t6 + t1
    } else if ((va < maxVelocity && sa > target) || (va > maxVelocity && sa > target && sv > target)) {
        //B & C.2
        println("B & C.2")
        t1 = (target / (2 * jerk)).pow(1.0 / 3.0)
        v1 = (jerk * t1.pow(2)) / 2.0
        a1 = jerk * t1
        p1 = (jerk * t1.pow(3)) / 6.0
        t2 = t1
        p2 = p1
        v2 = v1
        t3 = 2 * t1
        v3 = (t1 * a1) / 2.0
        p3 = target / 2
        t4 = t3
        v4 = v3
        p4 = p3
        t5 = 3 * t1
        v5 = v1
        p5 = target - p1
        t6 = t5
        v6 = v5
        p6 = p5 + (p2 - p1)
        t7 = 4 * t1
    } else if (va < maxVelocity && sa < target && sv < target) {
        //D.1
        println("D.1")
        t1 = maxAcceleration / jerk
        v1 = .5 * t1 * maxAcceleration
        p1 = jerk * (t1.pow(3) / 6.0)
        v2 = maxVelocity - v1
        t2 = t1 + (maxVelocity - maxAcceleration.pow(2) / jerk) / maxAcceleration
        p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2) / 2.0
        t3 = t1 + t2
        v3 = maxAcceleration
        p3 = p2 + v2 * (t3 - t2) + maxAcceleration * (t3 - t2).pow(2) / 2.0 - jerk * (t3 - t2).pow(3) / 6
        p4 = p3 + target - 2 * p3
        v4 = v3
        p5 = p4 + (p3 - p2)
        v5 = v2
        p6 = p5 + (p2 - p1)
        v6 = v1
        t4 = t3 + ((target - 2.0 * p3) / maxVelocity)
        t5 = t4 + t1
        t6 = t5 + (t2 - t1)
        t7 = t6 + t1
    } else {
        //D.2
        println("D.2")
        t1 = maxAcceleration / jerk
        v1 = (t1 * maxAcceleration) / 2
        p1 = (jerk * t1.pow(3)) / 6
        t2 =
            (.5) * (((4.0 * target * jerk.pow(2) + maxAcceleration.pow(3)) / (maxAcceleration * jerk.pow(2))).pow(.5) - maxAcceleration / jerk)
        v2 = v1 + maxAcceleration * (t2 - t1)
        p2 = p1 + v1 * (t2 - t1) + maxAcceleration * ((t2 - t1).pow(2)) / 2
        t3 = t2 + t1
        v3 = v2 + v1
        p3 = p2 + v2 * (t3 - t2) + maxAcceleration * (t3 - t2).pow(2) / 2 - (jerk * (t3 - t2).pow(3)) / 6
        t4 = t3
        v4 = v3
        p4 = p3
        t5 = t4 + t1
        v5 = v2
        p5 = p4 + (p3 - p2)
        t6 = t5 + (t2 - t1)
        v6 = v1
        p6 = p5 + (p2 - p1)
        t7 = t6 + t1
    }

    val timeslice = t7 / points

    println("$p1 $p2 $p3 $p4 $p5 $p6 $target")
    println("$t1 $t2 $t3 $t4 $t5 $t6 $t7")

    val timeList = ArrayList<Double>(points) { 0.0 }
    val acceleration = ArrayList<Double>(points) { 0.0 }
    val velocity = ArrayList<Double>(points) { 0.0 }
    val position = ArrayList<Double>(points) { 0.0 }
    for (i in 0..<points) {
        val time = timeslice * i
        if (time < t1) acceleration[i] = acceleration.getOrElse(i - 1) { 0.0 } + jerk * timeslice
        else if (time < t2) acceleration[i] = acceleration[i - 1]
        else if (time < t3) acceleration[i] = acceleration[i - 1] - jerk * timeslice
        else if (time < t4) acceleration[i] = 0.0
        else if (time < t5) acceleration[i] = acceleration[i - 1] - jerk * timeslice
        else if (time < t6) acceleration[i] = acceleration[i - 1]
        else acceleration[i] = acceleration[i - 1] + jerk * timeslice

        velocity[i] = velocity.getOrElse(i - 1) { 0.0 } + acceleration[i] * timeslice
        position[i] = position.getOrElse(i - 1) { 0.0 } + velocity[i] * timeslice
        timeList[i] = time
    }

    for (i in 0..<points) {
        position[i] = start + position[i] * (final - start).sign
    }

    return MotionResult(acceleration, velocity, position, timeList, timeslice, final)
}

fun <T> ArrayList(points: Int, function: () -> T): ArrayList<T> {
    val array = ArrayList<T>(points)
    for (i in 0..<points) {
        array.add(function())
    }

    return array
}