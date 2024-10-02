package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Arm(hardwareMap: HardwareMap) {
    private val arm = hardwareMap.get(Servo::class.java, "arm")

    init {
        arm.direction = Servo.Direction.REVERSE
    }

    fun up() {
        arm.position = 1.0
    }

    fun down() {
        arm.position = 0.0
    }
}
