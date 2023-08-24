package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.PIDTerms

@Config
private object ArmPIDTerms {
    @JvmField var PID_TERMS = PIDTerms(2.0)
}

class Arm(hardwareMap: HardwareMap) {
    private val right = hardwareMap.get(CRServo::class.java, "right")
    private val left = hardwareMap.get(CRServo::class.java, "leftservo")
//    private val hub = ControlHub(hardwareMap, hardwareMap.get("Expansion Hub 2") as LynxDcMotorController)
//    private val pid = PID(ArmPIDTerms.PID_TERMS, 0.0, 270.0, -1.0, 1.0)

//    private var targetDegrees: Double = 0.0
//    private var currentDegrees: Double = 0.0

    var power = 0.0

    init {
//        hub.setJunkTicks()
        left.direction = DcMotorSimple.Direction.REVERSE
    }

//    fun move(degrees: Double) {
//        targetDegrees = degrees.coerceIn(0.0, 270.0)
//    }

    fun update() {
//        hub.refreshBulkData()

//        currentDegrees = hub.getEncoderTicks(2).toDouble() * (360.0/8192.0)
//        val power = pid.calculate(targetDegrees, -currentDegrees)
//        pid.log()
//        FtcDashboard.getInstance().telemetry.addData("pv", currentDegrees)
//        FtcDashboard.getInstance().telemetry.addData("sp", targetDegrees)
//        FtcDashboard.getInstance().telemetry.addData("error", pid.previousError)
//        FtcDashboard.getInstance().telemetry.update()

        left.power = power
        right.power = power

//        println(this.toString())
        println(power)
    }

//    override fun toString(): String {
//        return "Arm: { targetDegrees: $targetDegrees, currentDegrees: $currentDegrees }"
//    }
}