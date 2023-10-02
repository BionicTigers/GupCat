package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms

class Slide(hardwareMap: HardwareMap) {
    val Left = hardwareMap.get(DcMotorEx::class.java, "SlideL")
    val Right = hardwareMap.get(DcMotorEx::class.java, "SlideR")

    var height = 0
        set(value: Int) {
            field = value.coerceIn(0, 800)
        }
//    val pid = PID(PIDTerms(), )

    fun update() {

    }
}