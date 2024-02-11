package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Claw(hardwareMap: HardwareMap) {
    private val claw = hardwareMap.get(Servo::class.java, "claw")
    private val blinkin = hardwareMap.get(RevBlinkinLedDriver::class.java, "LEDs")

    fun open() {
        claw.position = 1.0
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
    }
    fun close() {
        claw.position = 0.0
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
    }
}