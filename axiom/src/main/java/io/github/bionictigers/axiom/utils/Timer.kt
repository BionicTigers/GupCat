package io.github.bionictigers.axiom.utils

import io.github.bionictigers.axiom.commands.BaseCommandState
import kotlin.time.Duration

class Timer(val duration: Duration) {
    private var initialTime: Duration? = null
    private var calledOnce = false

    var isFinished = false
        private set

    fun update(state: BaseCommandState): Timer {
        requireNotNull(state.enteredAt) { "Command must be in scheduler to use timer" }

        val timeInScheduler = state.enteredAt!!.elapsedNow()

        if (initialTime == null) {
            initialTime = timeInScheduler
        } else if (timeInScheduler - initialTime!! >= duration) {
            isFinished = true
        }

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