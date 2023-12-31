package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utils.ControlHub

/** Runs a tape measure up to the rigging in order to lift the robot */
class Cables (hardwareMap: HardwareMap) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, "hangMotor")
    private val hub = ControlHub(hardwareMap, hardwareMap.get("Control Hub") as LynxDcMotorController)

    /** Runs the hanging mechanism up at 100% power */
    fun pull() {
        motor.power = 1.0
    }

    /** Runs the hanging mechanism down at 100% power */
    fun raise() {
        motor.power = -1.0
    }

    /** Stops the motor */
    fun stop() {
        motor.power = 0.0
    }
}