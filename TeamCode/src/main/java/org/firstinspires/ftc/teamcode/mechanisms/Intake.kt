package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

class Intake(hardwareMap: HardwareMap) {
    val motor = hardwareMap.get(DcMotorEx::class.java, "Intake")

    fun start() {
        motor.power = 1.0
    }

    fun stop() {
        motor.power = 0.0
    }
}
