package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
/**Runs the output with two servos, one on the right and one on the left, to deposit a pixel onto
 * the backboard*/
class Output (hardwareMap: HardwareMap) {
    //Creates servos
    val left = hardwareMap.get(Servo::class.java, "leftServo")
    val right = hardwareMap.get(Servo::class.java, "rightServo")

    //Opens the left side of the trapdoor
    fun openLeft() {
        left.setPosition(0.0)
    }
    //Opens the right side of the trapdoor
    fun openRight() {
        right.setPosition(0.7)
    }
    //Closes both sides of the trapdoor
    fun close() {
        left.setPosition(0.3)
        right.setPosition(0.4)
    }
}