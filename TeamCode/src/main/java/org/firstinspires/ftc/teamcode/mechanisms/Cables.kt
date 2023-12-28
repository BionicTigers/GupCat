package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
/** Runs a tape measure up to the rigging in order to lift the robot */
class Cables (hardwareMap: HardwareMap) {
    private val servo = hardwareMap.get(CRServo::class.java, "hangServo")

    /** Runs the tape measure up at 100% power */
    fun lift() {
        servo.power = -1.0
    }

    /** Runs the tape measure down at 100% power */
    fun lower() {
        servo.power = 1.0
    }

    /** Stops the motor */
    fun stop() {
        servo.power = 0.0
    }
}