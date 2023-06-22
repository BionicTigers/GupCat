package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import org.firstinspires.ftc.teamcode.utils.oldPID
import org.firstinspires.ftc.teamcode.utils.Robot

class Lift(hardwareMap: HardwareMap, private val robot: Robot) {

    private val hub = ControlHub(hardwareMap, hardwareMap.get("Control Hub") as LynxDcMotorController)

    private val maximumHeight = 800

    private val motors = mapOf(
        "right" to hardwareMap.get(DcMotorEx::class.java, "right"),
        "middle" to hardwareMap.get(DcMotorEx::class.java, "middle"),
        "left" to hardwareMap.get(DcMotorEx::class.java, "left")
    )
    private val limitSwitch = hardwareMap.get(DigitalChannel::class.java, "limitSwitch")

    private val pid = PID(PIDTerms(), 0.0, maximumHeight.toDouble(), -1.0, 1.0)

    var trim: Double = 0.0
    var killPower: Boolean = true
    var targetHeight = 0
        set(value) {
            field = value.coerceIn(0, maximumHeight) //Minimum to Maximum
        }

    init {
        motors["middle"]!!.direction = DcMotorSimple.Direction.REVERSE
        hub.setJunkTicks()
    }

    fun update() {
        hub.refreshBulkData()
        val power = pid.calculate(targetHeight + trim, hub.getEncoderTicks(1).toDouble())
        pid.log()

        motors.forEach {
            it.value.power = if (killPower) 0.0 else power
        }
    }
}