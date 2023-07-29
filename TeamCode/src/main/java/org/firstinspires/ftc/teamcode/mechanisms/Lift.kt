package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import org.firstinspires.ftc.teamcode.utils.Robot

@Config
private object LiftPIDTerms {
    @JvmField var PID_TERMS = PIDTerms(2.0)
}

class Lift(hardwareMap: HardwareMap, private val robot: Robot) {

    private val hub = ControlHub(hardwareMap, hardwareMap.get("Expansion Hub 2") as LynxDcMotorController)

    private val maximumHeight = 1060.0

    private val motors = mapOf(
        "right" to hardwareMap.get(DcMotorEx::class.java, "right"),
        "middle" to hardwareMap.get(DcMotorEx::class.java, "middle"),
        "left" to hardwareMap.get(DcMotorEx::class.java, "left")
    )
    private val limitSwitch = hardwareMap.get(DigitalChannel::class.java, "limitSwitch")

    private val pid = PID(LiftPIDTerms.PID_TERMS, 0.0, maximumHeight.toDouble(), -1.0, 1.0)

    var targetHeight = 0
        set(value) {
            field = value
            trim = trim
        }
    var trim: Double = 0.0
        set(value) {
            print(value)
            field = value.coerceIn(0.0, maximumHeight - targetHeight.toDouble())
        }
    var killPower: Boolean = false

    init {
        //Basic Setup (Changing motor Run Modes/Setting Direction/Setting Junk Ticks)
        for ((_, motor) in motors) {
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        motors["middle"]!!.direction = DcMotorSimple.Direction.REVERSE
        motors["left"]!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        hub.setJunkTicks()
    }

    fun update() {
        hub.refreshBulkData()
        val power = pid.calculate(targetHeight + trim, -hub.getEncoderTicks(1).toDouble())
        pid.log()

        motors.forEach {
            it.value.power = if (killPower) 0.0 else power
        }

        println(this.toString())
    }

    override fun toString(): String {
        return "Lift { targetHeight: $targetHeight, trim: $trim, ticks: ${-hub.getEncoderTicks(1).toDouble()} }"
    }
}