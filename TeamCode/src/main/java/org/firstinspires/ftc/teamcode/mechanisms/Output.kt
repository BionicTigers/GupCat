package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**
 * Runs the claw with one servo to deposit a pixel onto the backdrop
 */
class Output(hardwareMap: HardwareMap) {
    //Creates servos
    val holder = hardwareMap.get(Servo::class.java, "claw")
    var state = OutputState.Close

    enum class OutputState {
        Intake,
        Open,
        Close,
    }

    /**
     * Opens the claw for intake
     */
    fun intake() {
        holder.position = .4
        state = OutputState.Intake
    }

    /**
     * Opens the claw for output
     */
    fun open() {
        holder.position = .2
        state = OutputState.Open
    }

    /**
     * Closes the claw
     */
    fun close() {
        holder.position = 0.0
        state = OutputState.Close
    }
}