package io.github.bionictigers.input

import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.axiom.input.BaseButton

class Joystick(value: Vector2<Double>) : BaseButton<Vector2<Double>>(value) {
    val inDeadZone = ArrayList<() -> Unit>()
    val onMove = ArrayList<() -> Unit>()
    val continuous = ArrayList<() -> Unit>()

    fun inDeadZone(lambda: () -> Unit): Joystick {
        inDeadZone.add(lambda)
        return this
    }

    fun onMove(lambda: () -> Unit): Joystick {
        onMove.add(lambda)
        return this
    }

    fun continuous(lambda: () -> Unit): Joystick {
        continuous.add(lambda)
        return this
    }

    override fun update(value: Vector2<Double>) {
        super.update(value)
    }
}