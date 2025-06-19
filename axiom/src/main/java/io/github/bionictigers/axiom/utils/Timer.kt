package io.github.bionictigers.axiom.utils

import io.github.bionictigers.axiom.commands.BaseCommandState

class Timer(val duration: Time) {
    private var initialTime: Time? = null
    private var calledOnce = false

    var isFinished = false
        private set

    fun update(state: BaseCommandState): Timer {
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