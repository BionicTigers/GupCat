package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**
 * Launches drones from robot
 */
class Drone(hardwareMap: HardwareMap) {
    private val drone = hardwareMap.get(Servo::class.java, "drone")

    /**
     * Starts drone flywheel
     */
    fun start() {
        drone.position = 0.46
    }

    /**
     * Stops drone flywheel
     */
    fun stop() {
        drone.position = 0.5
    }
}