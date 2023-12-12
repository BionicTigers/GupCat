package org.firstinspires.ftc.teamcode.utils.input

class Trigger {
    var state = 0.0
    var deadzone = 0.4

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
    fun update(newState: Double) {
        if (state != newState && newState >= deadzone) //Start Callbacks
            for (callback in startCallbacks)
                callback.invoke()
        else if (state == newState && newState >= deadzone) //Hold Callbacks
            for (callback in holdCallbacks)
                callback.invoke()
        else if (state != newState && newState < deadzone) //End Callbacks
            for (callback in endCallbacks)
                callback.invoke()

        state = newState
    }
}