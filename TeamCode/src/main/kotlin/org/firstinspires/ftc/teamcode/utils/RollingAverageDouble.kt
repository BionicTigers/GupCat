package org.firstinspires.ftc.teamcode.utils

import io.github.bionictigers.axiom.commands.Display
import io.github.bionictigers.axiom.commands.Value
import java.util.LinkedList
import java.util.Queue

/**
 * Calculate a rolling average from a stream of numbers. Only the last 'size' elements will be
 * considered.
 */
class NewRollingAverage(var size: Int) : Display() {
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
    }

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

    override fun serialize(): Map<String, Value> {
        return mapOf("average" to Value(average, true))
    }
}