package org.firstinspires.ftc.teamcode.utils

/**
 * Stores a min and max value
 */
data class Range<T: Comparable<T>>(val min: T, val max: T) {
    fun within(value: T): Boolean {
        return value > min
    }

    fun clamp(value: T): T {
        return value.coerceIn(min, max)
    }
}