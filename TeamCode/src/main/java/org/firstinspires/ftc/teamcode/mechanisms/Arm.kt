package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**
 * Controls the arm at the end of the freight flow to get into final scoring position
 */
class Arm(hardwareMap: HardwareMap) {
    private val arm = hardwareMap.get(Servo::class.java, "arm")

    init {
        arm.direction = Servo.Direction.REVERSE
    }

    /**
     * Raises the arm into its storage position
     */
    fun up() {
        arm.position = 0.4
    }

    /**
     * Lowers the arm into scoring position
     */
    fun down() {
        arm.position = 0.0
    }
}