package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
/**
 * Launches drones from robot
 */
//TODO Melia: update this mechanism to work with a servo instead of a motor
class Drone(hardwareMap: HardwareMap) {
    private val drone = hardwareMap.get(DcMotorEx::class.java, "motor")

    /**
     * Starts drone flywheel
     */
    fun start() {
        drone.power = 1.0
    }

    /**
     * Stops drone flywheel
     */
    fun stop() {  //Stops drone flywheel
        drone.power = 0.0
    }
}