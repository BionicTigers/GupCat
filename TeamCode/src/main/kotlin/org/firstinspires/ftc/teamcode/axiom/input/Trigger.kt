package io.github.bionictigers.input

import org.firstinspires.ftc.teamcode.axiom.input.BaseButton

class Trigger(value: Double) : BaseButton<Double>(value) {
    enum class TriggerState {
        DOWN,
        UP,
        HOLD,
        NONE
    }

    var state = TriggerState.NONE
        private set

    val onDown = ArrayList<(Double) -> Unit>()
    val onUp = ArrayList<(Double) -> Unit>()
    val onHold = ArrayList<(Double) -> Unit>()

    fun onDown(lambda: (Double) -> Unit): Trigger {
        onDown.add(lambda)
        return this
    }

    fun onUp(lambda: (Double) -> Unit): Trigger {
        onUp.add(lambda)
        return this
    }

    override fun update(value: Double): List<(Double) -> Unit> {
        state = when {
            value > 0.5 && this.value <= 0.5 -> TriggerState.DOWN
            value <= 0.5 && this.value > 0.5 -> TriggerState.UP
            value > 0.5 -> TriggerState.HOLD
            else -> TriggerState.NONE
        }
        super.update(value)

        return when (state) {
            TriggerState.DOWN -> onDown
            TriggerState.UP -> onUp
            TriggerState.HOLD -> onHold
            else -> emptyList()
        }
    }
}