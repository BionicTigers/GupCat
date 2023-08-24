package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.Robot

class Transfer(hardwareMap: HardwareMap) {
    private val transfer = hardwareMap.get(DcMotorEx::class.java, "transfer")

    fun run() {
        transfer.power = 1.0
    }

    fun stop() {
        transfer.power = 0.0
    }
}