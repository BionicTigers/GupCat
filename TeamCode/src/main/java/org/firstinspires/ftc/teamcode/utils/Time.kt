package org.firstinspires.ftc.teamcode.utils

data class Time internal constructor(private val time: Double = 0.0) {
    companion object {
        fun fromSeconds(seconds: Double): Time {
            return Time(seconds)
        }

        fun fromMilliseconds(milliseconds: Double): Time {
            return Time(milliseconds / 1000.0)
        }

        fun fromNanoseconds(nanoseconds: Double): Time {
            return Time(nanoseconds / 1000000000.0)
        }
    }

    fun seconds(): Double {
        return time
    }

    fun milliseconds(): Double {
        return time * 1000.0
    }

    fun nanoseconds(): Double {
        return time * 1000000000.0
    }

    operator fun plus(deltaTime: Time): Time {
        return Time(time + deltaTime.time)
    }

    operator fun minus(deltaTime: Time): Time {
        return Time(time - deltaTime.time)
    }

    operator fun times(deltaTime: Time): Time {
        return Time(time * deltaTime.time)
    }

    operator fun div(deltaTime: Time): Time {
        return Time(time / deltaTime.time)
    }

    operator fun compareTo(otherTime: Time): Int {
        val diff = seconds() - otherTime.seconds()
        return if (diff == 0.0) 0
        else if (diff > 0.0) 1
        else -1
    }
}