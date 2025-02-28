package org.firstinspires.ftc.teamcode.utils

import io.github.bionictigers.io.github.bionictigers.axiom.web.Display
import java.util.LinkedList
import java.util.Queue

/**
 * Calculate a rolling average from a stream of numbers. Only the last 'size' elements will be
 * considered.
 */
class NewRollingAverage(var size: Int) {
    private val queue: Queue<Double> = LinkedList()
    private var total: Double = 0.0

    init {
        resize(size)
    }

    /**
     * Get the size
     * @return Size
     */
    fun size(): Int {
        return size
    }

    /**
     * Resize the rolling average
     *
     *
     * Side Effect: the rolling average is reset
     * @param size
     */
    fun resize(size: Int) {
        this.size = size
        queue.clear()
    }

    /**
     * Add a number to the rolling average
     * @param number
     */
    fun addNumber(number: Double) {
        if (queue.size >= size) {
            val last = queue.remove()
            total -= last
        }

        queue.add(number)
        total += number

        if (queue.isEmpty()) avg = 0.0
        avg = total / queue.size.toDouble()
    }

    @Display
    private var avg: Double = 0.0

    val average: Double
        /**
         * Get the rolling average
         * @return rolling average, if available; otherwise 0
         */
        get() {
            if (queue.isEmpty()) return 0.0
            return total / queue.size.toDouble()
        }

    /**
     * Reset the rolling average
     */
    fun reset() {
        queue.clear()
    }
}