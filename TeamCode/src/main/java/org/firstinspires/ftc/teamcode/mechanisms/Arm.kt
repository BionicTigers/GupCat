package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
class Arm(hardwareMap: HardwareMap, private val robot: Robot) {
    private val right = hardwareMap.get(CRServo::class.java, "right")
    private val left = hardwareMap.get(CRServo::class.java, "left")
    private val controlHub = ControlHub(hardwareMap, hardwareMap.get(LynxDcMotorController::class.java, "Control Hub"))
    private val pid = PID(PIDTerms(), 0.0, 270.0, -1.0, 1.0)

    private var targetDegrees: Double = 0.0
    private val angleOffset: Double = 0.0

    init {
        controlHub.setJunkTicks()
    }

    fun move(degrees: Double) {
        targetDegrees = degrees.coerceIn(0.0, 270.0) + angleOffset
    }

    fun update() {
        controlHub.refreshBulkData()

        val degrees = controlHub.getEncoderTicks(0).toDouble() * (360/8192)
        val power = pid.calculate(targetDegrees, degrees)
        pid.log()

        left.power = power
        right.power = power
    }
}