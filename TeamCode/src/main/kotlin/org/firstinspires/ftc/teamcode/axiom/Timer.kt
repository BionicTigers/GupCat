package org.firstinspires.ftc.teamcode.axiom

import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.utils.Time

class Timer(val duration: Time) {
    private var initialTime: Time? = null
    private var calledOnce = false

    var isFinished = false
        private set

    fun update(state: CommandState): Timer {
        if (initialTime == null) {
            initialTime = state.timeInScheduler
        } else if (state.timeInScheduler - initialTime!! >= duration) {
            isFinished = true
        }

        println(state.timeInScheduler - initialTime!!)

        return this
    }

    fun once(callback: () -> Unit): Timer {
        if (isFinished && !calledOnce) {
            callback()
        }

        return this
    }

    fun finished(callback: () -> Unit): Timer {
        if (isFinished) callback()
        return this
    }

    fun reset() {
        initialTime = null
        isFinished = false
        calledOnce = false
    }
}