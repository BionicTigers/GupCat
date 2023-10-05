package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
/** Runs a tape measure up to the rigging in order to lift the robot*/
class Cables (hardwareMap: HardwareMap) {
    val servo = hardwareMap.get(CRServo::class.java, "hangingServo")

    fun lift(){
        servo.power = 0.6
    }
    fun stop(){
        servo.power = 0.0
    }
}