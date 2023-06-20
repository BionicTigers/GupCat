package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import org.firstinspires.ftc.teamcode.utils.oldPID
import org.firstinspires.ftc.teamcode.utils.Robot

class Lift(hardwareMap: HardwareMap, private val robot: Robot) {
    private val maximumHeight = 800

    private val motors = mapOf(
        "top" to hardwareMap.get(DcMotorEx::class.java, "top"),
        "middle" to hardwareMap.get(DcMotorEx::class.java, "middle"),
        "bottom" to hardwareMap.get(DcMotorEx::class.java, "bottom")
    )
    private val limitSwitch = hardwareMap.get(DigitalChannel::class.java, "limitSwitch")

    private val pid = PID(PIDTerms(), -1.0, 1.0, 0.0, maximumHeight.toDouble())

    var trim: Double = 0.0
    var killPower: Boolean = true
    var targetHeight = 0
        set(value) {
            field = value.coerceIn(0, maximumHeight) //Minimum to Maximum
        }

    init {
        motors["middle"]!!.direction = DcMotorSimple.Direction.REVERSE
    }

    fun update() {
        val power = pid.calculate(targetHeight + trim, 0.0)

        motors.forEach {
            it.value.power = if (killPower) 0.0 else power
        }
    }
}