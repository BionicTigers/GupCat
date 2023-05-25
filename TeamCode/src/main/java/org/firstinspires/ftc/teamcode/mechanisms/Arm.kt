package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.Robot
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utils.PID

class Arm (hardwareMap: HardwareMap, private val robot: Robot) {
    private val right = hardwareMap.get(CRServo::class.java, "right")
    private val left = hardwareMap.get(CRServo::class.java, "left")
    val pid = PID(1.0, 0.0, 0.0, -1.0, 1.0, 0.0, 270.0)

    private var targetDegrees: Double = 0.0
    private val angleOffset: Int = 8
    var power = 0

    fun move(degrees: Double) {
        targetDegrees = 0.0.coerceAtLeast(270.0.coerceAtMost(degrees)) - angleOffset
    }

    fun update() {
        left.power
        right.power
        pid.calculate(90.0, 0.0)
    }
}