package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**
 * Runs the claw with one servo to deposit a pixel onto the backdrop
 */
class Output (hardwareMap: HardwareMap) {
    //Creates servos
    val claw = hardwareMap.get(Servo::class.java, "claw")

    /**
     * Opens the claw
     */
    fun open() {
        claw.position = 0.8
    }

    /**
     * Closes the claw
     */
    fun close() {
        claw.position = 0.1
    }
}