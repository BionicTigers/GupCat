package io.github.bionictigers.input

import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.axiom.input.BaseButton

class Joystick(value: Vector2<Double>) : BaseButton<Vector2<Double>>(value) {
    enum class JoystickState {
        IN_DEAD_ZONE,
        ON_MOVE,
        CONTINUOUS
    }

    var deadzone = Vector2(.1, .1)

    var state = JoystickState.CONTINUOUS
        private set

    val inDeadZone = ArrayList<(Vector2<Double>) -> Unit>()
    val onMove = ArrayList<(Vector2<Double>) -> Unit>()
    val continuous = ArrayList<(Vector2<Double>) -> Unit>()

    fun inDeadZone(lambda: (Vector2<Double>) -> Unit): Joystick {
        inDeadZone.add(lambda)
        return this
    }

    fun onMove(lambda: (Vector2<Double>) -> Unit): Joystick {
        onMove.add(lambda)
        return this
    }

    fun continuous(lambda: (Vector2<Double>) -> Unit): Joystick {
        continuous.add(lambda)
        return this
    }

    override fun update(value: Vector2<Double>): List<(Vector2<Double>) -> Unit> {
        state = when {
            value.x in -deadzone.x..deadzone.x && value.y in -deadzone.y..deadzone.y -> JoystickState.IN_DEAD_ZONE
            value.x != 0.0 || value.y != 0.0 -> JoystickState.ON_MOVE
            else -> JoystickState.CONTINUOUS
        }

        super.update(value)

        return when (state) {
            JoystickState.IN_DEAD_ZONE -> listOf(inDeadZone, continuous).flatten()
            JoystickState.ON_MOVE -> listOf(onMove, continuous).flatten()
            JoystickState.CONTINUOUS -> continuous
        }
    }
}