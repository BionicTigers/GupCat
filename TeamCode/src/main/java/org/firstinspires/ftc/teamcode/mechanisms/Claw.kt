package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

import org.firstinspires.ftc.teamcode.utils.Robot

class Claw(hardwareMap: HardwareMap, private val robot: Robot) {
    private val claw = hardwareMap.get(Servo::class.java, "claw")

    fun open() {
        claw.position = 1.0
    }
    fun close() {
        claw.position = 0.0
    }
}