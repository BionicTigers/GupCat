package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms

class Slide(hardwareMap: HardwareMap) {
    val left = hardwareMap.get(DcMotorEx::class.java, "SlideL")
    val right = hardwareMap.get(DcMotorEx::class.java, "SlideR")
    val pid = PID(PIDTerms(), 0.0, 1000.0, -1.0, 1.0)
    private val hub = ControlHub(hardwareMap, hardwareMap.get("Control Hub") as LynxDcMotorController)

    var height = 0.0
        set(value: Double) {
            field = value.coerceIn(0.0, 800.0)
        }
//    val pid = PID(PIDTerms(), )
    init {
        hub.setJunkTicks()
        right.direction = DcMotorSimple.Direction.REVERSE
    }

    fun update() {
        val encoderTicks = hub.getEncoderTicks(0).toDouble()
        val power = pid.calculate(height, encoderTicks)
        left.power = power
        right.power = power

    }
}