package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**Runs the output with two servos, one on the right and one on the left, to deposit a pixel onto
 * the backdrop*/
class Output (hardwareMap: HardwareMap) {
    //Creates servos
    val claw = hardwareMap.get(Servo::class.java, "claw")

    /**
     * Opens the left side of the trapdoor
     */
    fun open() {
        claw.position = 0.8
    }

    /**
     * Closes both sides of the trapdoor
     */
    fun close() {
        claw.position = 0.1
    }
}