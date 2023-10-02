package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
/**
 * launches drones from robot
 */
class Drone(hardwareMap: HardwareMap) {
    val drone = hardwareMap.get(DcMotorEx::class.java, "motor")

    fun start() {
        drone.power = 1.0
    }
    fun stop() {
        drone.power = 0.0
    }
}