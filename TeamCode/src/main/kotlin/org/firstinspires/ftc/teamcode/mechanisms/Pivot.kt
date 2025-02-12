package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
import org.firstinspires.ftc.teamcode.input.Gamepad
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Encoder
import org.firstinspires.ftc.teamcode.utils.Persistents
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
                override val pid = PID(PIDTerms(1.5, 90.0), 0.0, 1800.0, -1.0, 1.0)
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

//    val upPIDTerms = interpolatedMapOf(
//        0.0 to 4.75,
//        1500.0 to 1.0
//    )
//
//    val downPIDTerms = interpolatedMapOf(
//        1500.0 to 4.0,
//        500.0 to 3.0,
//        0.0 to 1.0
//    )

    val offset = interpolatedMapOf(
        0.0 to 0.0,
        1.0 to -0.15,
        25.0 to 0.2,
        750.0 to 0.25,
        1500.0 to 0.075
    )

    var ticks = 0

    override val dependencies: List<System> = emptyList()
    override val beforeRun = Command(PivotState.default(hardwareMap.getByName("pivot"), hardwareMap.getByName("pivot2"), exHub.getEncoder(1)))
        .setOnEnter {
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.motor.power = 0.0
            it.motor.direction = DcMotorSimple.Direction.REVERSE
            it.motor2.power = 0.0
//            it.motor2.direction = DcMotorSimple.Direction.REVERSE

            it.pid.reset()

//            println(Persistents.pivotTicks)
            if (Persistents.pivotTicks == null) Persistents.pivotTicks = exHub.rawGetEncoderTicks(3)
            exHub.setJunkTicks(3, Persistents.pivotTicks)
        }
        .setAction {
            exHub.refreshBulkData()
            ticks = exHub.getEncoderTicks(3)

//            if (it.targetPosition >= ticks)
//                it.pid.kP = upPIDTerms[ticks.toDouble()]
//            else
//                it.pid.kP = downPIDTerms[ticks.toDouble()]

            val power = it.pid.calculate(it.targetPosition.toDouble(), ticks.toDouble()) + offset[ticks.toDouble()]
            if (limitSwitch.state || it.targetPosition > 0) {
                it.motor.power = power
                it.motor2.power = power
            } else {
                it.motor2.power = -.05
                it.motor.power = -.05
            }

            if (!limitSwitch.state) {
                exHub.setJunkTicks()
                Persistents.pivotTicks = exHub.rawGetEncoderTicks(3)
            }

//            println(Persistents.pivotTicks)

//            telemetry.addData("Pivot Process P-VAL", it.pid.kP)
//            telemetry.addData("Pivot Process Value", exHub.getEncoderTicks(3))
//            telemetry.addData("Pivot Set Point", it.targetPosition)
//            telemetry.update()
            false
        }
    override val afterRun = null

    val max = 1800

    //TODO: Swap to an angle
    var pivotTicks: Int
        set(value) {
            beforeRun.state.targetPosition = value.coerceIn(-100,max)
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

    fun log(telemetry: Telemetry) {
        telemetry.addData("pivotTicks", pivotTicks)
        telemetry.addData("pivotActualTicks", ticks)
    }
}