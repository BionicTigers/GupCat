package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
/** Runs a tape measure up to the rigging in order to lift the robot */
class Cables (hardwareMap: HardwareMap) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, "hangMotor")

    /** Runs the tape measure up at 60% power */
    fun lift() {
        motor.power = 0.6
    }

    /** Runs the tape measure down at 60% power */
    fun lower() {
        motor.power = -0.6
    }

    /** Stops the servo */
    fun stop() {
        motor.power = 0.0
    }
}