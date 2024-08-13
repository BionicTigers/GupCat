package io.github.bionictigers.input

import org.firstinspires.ftc.teamcode.axiom.input.BaseButton

class Trigger(value: Double) : BaseButton<Double>(value) {
    val onDown = ArrayList<() -> Unit>()
    val onUp = ArrayList<() -> Unit>()
    val onHold = ArrayList<() -> Unit>()

    fun onDown(lambda: () -> Unit): Trigger {
        onDown.add(lambda)
        return this
    }

    fun onUp(lambda: () -> Unit): Trigger {
        onUp.add(lambda)
        return this
    }

    override fun update(value: Double) {
        super.update(value)
    }
}