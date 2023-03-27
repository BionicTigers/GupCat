package org.firstinspires.ftc.teamcode.utils.input

import com.qualcomm.robotcore.hardware.Gamepad

class BoolButton() {
    var state: Boolean = false

    //Assign sets for the callbacks
    private val startCallbacks = mutableSetOf<(() -> Unit)>()
    private val holdCallbacks = mutableSetOf<(() -> Unit)>()
    private val endCallbacks = mutableSetOf<(() -> Unit)>()

    //Whenever the button is initially pressed
    fun onStart(callback: () -> Unit) {
        startCallbacks.add(callback)
    }

    //Repeatably call whenever the button is held down
    fun onHold(callback: () -> Unit) {
        holdCallbacks.add(callback)
    }

    //Call whenever the button is let go of
    fun onEnd(callback: () -> Unit) {
        endCallbacks.add(callback)
    }

    //Update State and call Callbacks
    fun update(newState: Boolean) {
        if (state != newState && newState) //Start Callbacks
            for (callback in startCallbacks)
                callback.invoke()
        else if (state == newState && newState) //Hold Callbacks
            for (callback in holdCallbacks)
                callback.invoke()
        else if (state != newState) //End Callbacks
            for (callback in endCallbacks)
                callback.invoke()

        state = newState
    }
}