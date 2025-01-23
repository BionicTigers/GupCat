package org.firstinspires.ftc.teamcode.motion

import org.firstinspires.ftc.teamcode.utils.Time
import kotlin.math.abs
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

fun generateMotionProfile(
    start: Number,
    final: Number,
    jerk: Number,
    maxAcceleration: Number,
    maxVelocity: Number,
    startingVelocity: Number? = null,
    points: Int = 600
) = generateMotionProfile(start.toDouble(), final.toDouble(), jerk.toDouble(), maxAcceleration.toDouble(), maxVelocity.toDouble(), startingVelocity?.toDouble(), points)

/**
 * Generate a motion profile based off certain parameters
 *
 * @param jerk mm/s^3
 * @param maxAcceleration mm/s^2
 * @param maxVelocity mm/s
 */
private fun generateMotionProfile(
    s: Double,
    f: Double,
    jerk: Double,
    maxAcceleration: Double,
    maxVelocity: Double,
    startingVelocity: Double? = null,
    points: Int = 600
): MotionResult {
    var start = s
    var final = f

    val inverse = final - start < 0

    if (inverse) {
        start = f
        final = s
    }

    if (final - start == 0.0) return MotionResult(listOf(0.0), listOf(0.0), listOf(start), listOf(0.0), 1.0, start)

    val va = maxAcceleration.pow(2) / jerk
    val sa = 2.0 * maxAcceleration.pow(3) / jerk.pow(2)
    val sv =
        if (maxVelocity * jerk < maxAcceleration.pow(2)) 2 * maxVelocity * sqrt(maxVelocity / jerk) else maxVelocity * (maxVelocity / maxAcceleration + maxAcceleration / jerk)
    var vMax = maxVelocity

    var t0 = 0.0
    var t1: Double
    var t2: Double
    var t3: Double
    val t4: Double
    val t5: Double
    val t6: Double
    val t7: Double

    var p0 = 0.0
    var p1: Double
    var p2: Double
    var p3: Double
    val p4: Double
    val p5: Double
    val p6: Double

    var v0: Double? = startingVelocity
    var v1: Double
    var v2: Double
    var v3: Double
    val v4: Double
    val v5: Double
    val v6: Double

    var a0 = 0.0
    var a1: Double
    val a2: Double
    val a3: Double
    val a4: Double
    val a5: Double
    val a6: Double

    println("$final $start")
    var target = abs(final - start)

    if (v0 != null) {
        if ((va > maxVelocity && sa < target) || (va > maxVelocity && sa > target && sv < target)) {
            //A & C.1
            println("A & C.1")
            t1 = (maxVelocity / jerk).pow(.5)
            p1 = (jerk * t1.pow(3)) / 6.0
            v1 = (jerk * t1.pow(2)) / 2.0
            a1 = jerk * t1
            t2 = t1
            v2 = v1
            p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2) / 2
            a2 = a1
            t3 = 2 * t1
            v3 = maxVelocity
            p3 = maxVelocity * (maxVelocity / jerk).pow(.5)
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
            a2 = a1
            t3 = 2 * t1
            v3 = (t1 * a1) / 2.0
            p3 = target / 2
        } else if (va < maxVelocity && sa < target && sv < target) {
            //D.1
            println("D.1")
            t1 = maxAcceleration / jerk
            v1 = .5 * t1 * maxAcceleration
            p1 = jerk * (t1.pow(3) / 6.0)
            a1 = maxAcceleration
            v2 = maxVelocity - v1
            t2 = t1 + (maxVelocity - maxAcceleration.pow(2) / jerk) / maxAcceleration
            p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2) / 2.0
            a2 = maxAcceleration
            t3 = t1 + t2
            v3 = maxVelocity
            p3 = p2 + v2 * (t3 - t2) + maxAcceleration * (t3 - t2).pow(2) / 2.0 - jerk * (t3 - t2).pow(3) / 6
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
            a2 = maxAcceleration
            t3 = t2 + t1
            v3 = v2 + v1
            p3 = p2 + v2 * (t3 - t2) + maxAcceleration * (t3 - t2).pow(2) / 2 - (jerk * (t3 - t2).pow(3)) / 6
        }

        if (v0 > v3) { v0 = v3 }

        if (v0 < v1) {
            t0 = (2 * v0 / jerk).pow(.5)
            a0 = jerk * t0
            p0 = jerk * (t0.pow(3)) / 6
        } else if (v0 < v2) {
            a0 = maxAcceleration
            t0 = t1 + (v0 - v1) / maxAcceleration
            p0 = p1 + v1 * (t0 - t1) + maxAcceleration * (t0 - t1).pow(2) / 2
        } else if (v0 < v3) {
            t0 = t3 - ((2 / jerk) * (v3 - v0)).pow(.5)
            a0 = a2 - (t0 - t2) * jerk
            p0 = p2 + v2 * (t0 - t2) + a2 * (t0 - t2).pow(2) / 2 - jerk * (t0 - t2).pow(3) / 6
        } else {
            t0 = t3
            a0 = 0.0
            p0 = p3
        }

        target += p0
        vMax = v3

    } else {
        v0 = 0.0
    }


    if ((va > vMax && sa < target) || (va > vMax && sa > target && sv < target)) {
        //A & C.1
        println("A & C.1")
        t1 = (vMax / jerk).pow(.5)
        p1 = (jerk * t1.pow(3)) / 6.0
        v1 = (jerk * t1.pow(2)) / 2.0
        t2 = t1
        v2 = v1
        p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2) / 2
        t3 = 2 * t1
        v3 = vMax
        p3 = vMax * (vMax / jerk).pow(.5)
        t4 = t3 + (target - 2 * p3) / vMax
        v4 = vMax
        p4 = p3 + target - 2 * p3
        t5 = t4 + t1
        v5 = v4 - v1
        p5 = target - p1
        t6 = t5
        v6 = v5
        p6 = p5
        t7 = t6 + t1
    } else if ((va < vMax && sa > target) || (va > vMax && sa > target && sv > target)) {
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
    } else if (va < vMax && sa < target && sv < target) {
        //D.1
        println("D.1")
        t1 = maxAcceleration / jerk
        v1 = .5 * t1 * maxAcceleration
        p1 = jerk * (t1.pow(3) / 6.0)
        v2 = vMax - v1
        t2 = t1 + (vMax - maxAcceleration.pow(2) / jerk) / maxAcceleration
        p2 = p1 + v1 * (t2 - t1) + maxAcceleration * (t2 - t1).pow(2) / 2.0
        t3 = t1 + t2
        v3 = maxVelocity
        p3 = p2 + v2 * (t3 - t2) + maxAcceleration * (t3 - t2).pow(2) / 2.0 - jerk * (t3 - t2).pow(3) / 6
        p4 = p3 + target - 2 * p3
        v4 = v3
        p5 = p4 + (p3 - p2)
        v5 = v2
        p6 = p5 + (p2 - p1)
        v6 = v1
        t4 = t3 + ((target - 2.0 * p3) / vMax)
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
//
//    println("$p1 $p2 $p3 $p4 $p5 $p6 $target")
//    println("$t1 $t2 $t3 $t4 $t5 $t6 $t7")
//    println("$v1 $v2 $v3 $v4 $v5 $v6")

    val timeList = ArrayList(600) { 0.0 }
    val acceleration = ArrayList(600) { 0.0 }
    val velocity = ArrayList(600) { 0.0 }
    val position = ArrayList(600) { 0.0 }
    println("t0: $t0, t7: $t7")
    for (i in 0..<points) {
        val time = t0 + timeslice * i
        if (time < t1) acceleration[i] = acceleration.getOrElse(i - 1) { 0.0 } + jerk * timeslice
        else if (time < t2) acceleration[i] = acceleration.getOrElse(i - 1) { a0 }
        else if (time < t3) acceleration[i] = acceleration.getOrElse(i - 1) { a0 } - jerk * timeslice
        else if (time < t4) acceleration[i] = 0.0
        else if (time < t5) acceleration[i] = acceleration.getOrElse(i - 1) { a0 } - jerk * timeslice
        else if (time < t6) acceleration[i] = acceleration.getOrElse(i - 1) { a0 }
        else acceleration[i] = acceleration.getOrElse(i - 1) { a0 } + jerk * timeslice

        velocity[i] = velocity.getOrElse(i - 1) { 0.0 } + acceleration[i] * timeslice
        position[i] = position.getOrElse(i - 1) { 0.0 } + velocity[i] * timeslice
        timeList[i] = time
        if (position[i] >= final - start || time >= t7) {
            position.trimZeros()
            timeList.trimZeros()
            velocity.trimZeros()
            acceleration.trimZeros()
            break
        }
    }

    for (i in 0..<position.size) {
        position[i] = start + position[i] * (final - start).sign
    }

    if (inverse) {
        acceleration.reverse()
        velocity.reverse()
        position.reverse()
    }
    println("p0: $p0, p7: ${position.last()}")

    return MotionResult(acceleration, velocity, position, timeList, timeslice, final)
}

fun <T> ArrayList<T>.trimZeros() {
    var i = 0
    while (i < this.size) {
        if (this[i] == 0.0) {
            this.removeAt(i)
        } else {
            i++
        }
    }
}

fun <T> ArrayList(points: Int, function: () -> T): ArrayList<T> {
    val array = ArrayList<T>(points)
    for (i in 0..<points) {
        array.add(function())
    }

    return array
}