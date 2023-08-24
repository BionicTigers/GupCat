package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

import org.firstinspires.ftc.teamcode.utils.Robot

class Claw(hardwareMap: HardwareMap) {
    private val claw = hardwareMap.get(Servo::class.java, "claw")
    private val distSensor = hardwareMap.get(DistanceSensor::class.java, "distSensor")

    private var cycles = 0
    private var dist = 0.0

    fun open() {
        claw.position = 1.0
    }

    fun close() {
        claw.position = 0.0
    }

    @SuppressWarnings("WeakerAccess")
    fun getDist(): Double { //gets the distance every 10 cycles
        cycles++
        if (cycles == 10) {
            dist = distSensor.getDistance(DistanceUnit.CM)
            cycles = 0
        }
        return dist
    }
    fun coneDetected(): Boolean {return getDist() < 5}

}