package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 *  Brings pixels into the robot
 */
class Intake(hardwareMap: HardwareMap) {
    private val colorSensor = hardwareMap.get(ColorSensor::class.java, "Sensor")
    private val motor = hardwareMap.get(DcMotorEx::class.java, "Intake")

    fun start() {  //Spins the intake
        motor.power = 1.0
    }

    fun stop() {  //Stops the intake
        motor.power = 0.0
    }
}