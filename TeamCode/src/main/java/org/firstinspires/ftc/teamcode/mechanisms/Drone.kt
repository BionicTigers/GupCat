package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
/**
 * launches drones from robot
 */
class Drone(hardwareMap: HardwareMap) {
    private val drone = hardwareMap.get(DcMotorEx::class.java, "motor")

    fun start() {  //Starts drone flywheel
        drone.power = 0.8
    }
    fun stop() {  //Stops drone flywheel
        drone.power = 0.0
    }
}