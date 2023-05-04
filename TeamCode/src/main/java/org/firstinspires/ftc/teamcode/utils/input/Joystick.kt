package org.firstinspires.ftc.teamcode.utils.input

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.utils.Vector2
import kotlin.math.abs

class Joystick {
    var state: Vector2? = null
    private var deadzone: Float = 0.05f

    private val changeCallbacks = mutableSetOf<((Vector2) -> Unit)>()
    private val restCallbacks = mutableSetOf<((Vector2) -> Unit)>()
    private val restOnceCallbacks = mutableSetOf<((Vector2) -> Unit)>()

    //Call whenever there is a change in the position
    fun onChange(callback: (Vector2) -> Unit) {
        changeCallbacks.add(callback)
    }

    //Repeatably call whenever the joystick is in rest position
    fun onRest(callback: (Vector2) -> Unit) {
        restCallbacks.add(callback)
    }

    //Call once at the start of entering rest position
    fun onRestOnce(callback: (Vector2) -> Unit) {
        restOnceCallbacks.add(callback)
    }

    //Update State and call Callbacks
    fun update(newState: Vector2) {

        if (state != newState && abs(newState.magnitude()) <= deadzone) { //Rest Once Callbacks
            for (callback in restOnceCallbacks)
                callback.invoke(newState)
        }

        if (abs(newState.magnitude()) < deadzone) {//Rest Callbacks
            for (callback in restCallbacks)
                callback.invoke(newState)
        }

        if (state != newState) {//Changed Callbacks
            for (callback in changeCallbacks)
                callback.invoke(newState)
        }

        state = newState
    }
}