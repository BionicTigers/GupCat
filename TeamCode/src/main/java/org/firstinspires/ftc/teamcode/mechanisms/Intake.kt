package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 *  Brings pixels into the robot
 */
class Intake(hardwareMap: HardwareMap) {
//    private val colorSensor = hardwareMap.get(ColorSensor::class.java, "sensor")
    private val intakeMotor = hardwareMap.get(DcMotorEx::class.java, "intake")
    private val transferMotor = hardwareMap.get(DcMotorEx::class.java, "transfer")

    fun startBoth() {
        intakeMotor.power = 0.8
        transferMotor.power = 0.4
    }

    fun startTransfer() {
        transferMotor.power = 0.6
    }

    fun stopIntake() {
        intakeMotor.power = 0.0
    }

    fun stopTransfer() {
        transferMotor.power = 0.0
    }

    fun reverse() {
        intakeMotor.power = -0.6
        transferMotor.power = -1.0
    }
}