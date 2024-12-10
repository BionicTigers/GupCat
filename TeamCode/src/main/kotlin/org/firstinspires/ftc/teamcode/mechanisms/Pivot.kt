package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.System
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Encoder
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.interpolatedMapOf

interface PivotState : CommandState {
    val encoder: Encoder
    var targetPosition: Int
    val pid: PID
    val motor: DcMotorEx
    val motor2: DcMotorEx

    companion object {
        fun default(motor: DcMotorEx, motor2: DcMotorEx, encoder: Encoder): PivotState {
            return object : PivotState, CommandState by CommandState.default("Pivot") {
                override val encoder = encoder
                override var targetPosition = 0
                override val pid = PID(PIDTerms(2.0, 0.0), 0.0, 1000.0, -1.0, 1.0)
                override val motor = motor
                override val motor2 = motor2
            }
        }
    }
}

class Pivot(hardwareMap: HardwareMap) : System {
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
    val limitSwitch = hardwareMap.getByName<DigitalChannel>("pivotSwitch")

    private val telemetry = FtcDashboard.getInstance().telemetry

    val upPIDTerms = interpolatedMapOf(
        0.0 to 1.75,
        1500.0 to 1.0
    )

    val downPIDTerms = interpolatedMapOf(
        1500.0 to 2.0,
        500.0 to 1.0,
        0.0 to 1.0
    )

    val offset = interpolatedMapOf(
        0.0 to 0.0,
        1.0 to -0.15,
        25.0 to 0.2,
        750.0 to 0.25,
        1500.0 to 0.075
    )

    init {
        exHub.refreshBulkData()
        exHub.setJunkTicks(3, exHub.getEncoderTicks(3) - 80)
    }

    override val dependencies: List<System> = emptyList()
    override val beforeRun = Command(PivotState.default(hardwareMap.getByName("pivot"), hardwareMap.getByName("pivot2"), exHub.getEncoder(1)))
        .setOnEnter {
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.motor.power = 0.0
            it.motor2.power = 0.0

            it.pid.reset()
        }
        .setAction {
            exHub.refreshBulkData()

            if (it.targetPosition >= exHub.getEncoderTicks(3).toDouble())
                it.pid.kP = upPIDTerms[exHub.getEncoderTicks(3).toDouble()]
            else
                it.pid.kP = downPIDTerms[exHub.getEncoderTicks(3).toDouble()]

            val power = it.pid.calculate(it.targetPosition.toDouble(), exHub.getEncoderTicks(3).toDouble()) + offset[exHub.getEncoderTicks(3).toDouble()]
            if (limitSwitch.state || it.targetPosition > 0) {
                it.motor.power = power
                it.motor2.power = power
            } else {
                it.motor2.power = -.1
                it.motor.power = -.1
            }
            if (!limitSwitch.state)
                exHub.setJunkTicks()

//            telemetry.addData("Pivot Process P-VAL", it.pid.kP)
//            telemetry.addData("Pivot Process Value", exHub.getEncoderTicks(3))
//            telemetry.addData("Pivot Set Point", it.targetPosition)
//            telemetry.update()
            false
        }
    override val afterRun = null

    //TODO: Swap to an angle
    var pivotTicks: Int
        set(value) {
            beforeRun.state.targetPosition = value.coerceIn(-100,1600)
        }
        get() = beforeRun.state.targetPosition

    val currentPosition: Int
        get() = exHub.getEncoderTicks(3)

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.leftTrigger.onHold {
            pivotTicks -= (1500 * Scheduler.loopDeltaTime.seconds() * it).toInt()
        }

        gamepad.rightTrigger.onHold {
            pivotTicks += (1500 * Scheduler.loopDeltaTime.seconds() * it).toInt()
        }
    }
}