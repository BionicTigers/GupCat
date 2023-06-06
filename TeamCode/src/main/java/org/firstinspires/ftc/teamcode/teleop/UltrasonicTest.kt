package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.UltrasonicSensor

@TeleOp(name="Ultrasonic Test")
class UltrasonicTest : LinearOpMode() {
    override fun runOpMode() {
        val side1 = hardwareMap.get(UltrasonicSensor::class.java, "us1")

        println(side1.ultrasonicLevel)
    }

}