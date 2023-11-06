package org.firstinspires.ftc.teamcode.utils.vision

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

class ServoTest (hardwareMap: HardwareMap) {
    private val servo = hardwareMap.get(CRServo::class.java, "servo")

    fun run() {
        servo.power = 1.0
    }
    fun backward() {
        servo.power = -1.0
    }
    fun stop() {
        servo.power = 0.0
    }
}