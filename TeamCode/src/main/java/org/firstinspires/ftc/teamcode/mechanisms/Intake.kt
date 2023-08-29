package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.vision.Color
import org.firstinspires.ftc.teamcode.utils.vision.VisionConstants

class Intake(hardwareMap: HardwareMap) {
    val colorSensor = hardwareMap.get(ColorSensor::class.java, "Sensor")
    val motor = hardwareMap.get(DcMotorEx::class.java, "Intake")

    fun start() {
        motor.power = 1.0
    }

    fun stop() {
        motor.power = 0.0
    }
}