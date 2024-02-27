package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**
 * Allows extra motion between arm and slides in order to create more versatility in scoring positions
 */
class Chainbar(hardwareMap: HardwareMap) {
    private val leftServo = hardwareMap.get(Servo::class.java, "chainbarLeft")
    private val rightServo = hardwareMap.get(Servo::class.java, "chainbarRight")

    /**
     * Sets one servo to reverse to prevent conflicts
     */
    init {
        rightServo.direction = Servo.Direction.REVERSE
    }

    /**
     * Rotates the arm up (towards sky)
     */
    fun down() {
        leftServo.position = 0.15
        rightServo.position = 0.15
    }

    /**
     * Rotates the arm down (towards transfer)
     */
    fun up() {
        leftServo.position = 1.0
        rightServo.position = 1.0
    }
}