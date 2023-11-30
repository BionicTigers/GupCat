package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**Runs the output with two servos, one on the right and one on the left, to deposit a pixel onto
 * the backdrop*/
class Output (hardwareMap: HardwareMap) {
    //Creates servos
    val left = hardwareMap.get(Servo::class.java, "outputLeft")
    val right = hardwareMap.get(Servo::class.java, "outputRight")

    /**
     * Opens the left side of the trapdoor
     */
    fun openLeft() {
        left.position = 0.0
    }

    /**
     * Opens the right side of the trapdoor
     */
    fun openRight() {
        right.position = 0.7
    }

    /**
     * Closes both sides of the trapdoor
     */
    fun close() {
        left.position = 0.3
        right.position = 0.4
    }
}