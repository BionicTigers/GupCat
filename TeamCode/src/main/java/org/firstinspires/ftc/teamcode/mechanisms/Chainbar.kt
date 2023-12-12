package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Chainbar(hardwareMap: HardwareMap) {
    private val leftServo = hardwareMap.get(Servo::class.java, "chainbarLeft")
    private val rightServo = hardwareMap.get(Servo::class.java, "chainbarRight")

    init {
        rightServo.direction = Servo.Direction.REVERSE
    }

    fun up() {
        leftServo.position = 1.0
        rightServo.position = 1.0
    }

    fun down() {
        leftServo.position = 0.0
        rightServo.position = 0.0
    }
}