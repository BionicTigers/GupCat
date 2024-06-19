package org.firstinspires.ftc.teamcode.utils

data class Time internal constructor(private val time: Double = 0.0) {

    companion object {
        fun fromSeconds(seconds: Double): Time {
            return Time(seconds)
        }

        fun fromMilliseconds(milliseconds: Double): Time {
            return Time(milliseconds / 1000.0)
        }

        fun fromNanoseconds(minutes: Double): Time {
            return Time(minutes * 60.0)
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

    operator fun plus(otherTime: Time): Time {
        return Time(time + otherTime.time)
    }

    operator fun minus(otherTime: Time): Time {
        return Time(time - otherTime.time)
    }

    operator fun times(otherTime: Time): Time {
        return Time(time * otherTime.time)
    }

    operator fun div(otherTime: Time): Time {
        return Time(time / otherTime.time)
    }

    operator fun compareTo(otherTime: Time): Int {
        val diff = seconds() - otherTime.seconds()
        return if (diff == 0.0) 0
        else if (diff > 0.0) 1
        else -1
    }
}