package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
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

interface PivotState : CommandState {
    val encoder: Encoder
    var targetPosition: Int
    val pid: PID
    val motor: DcMotorEx

    companion object {
        fun default(motor: DcMotorEx, encoder: Encoder): PivotState {
            return object : PivotState, CommandState by CommandState.default("Pivot") {
                override val encoder = encoder
                override var targetPosition = 0
                override val pid = PID(PIDTerms(4.0, 0.0), 0.0, 1000.0, -1.0, 1.0)
                override val motor = motor
            }
        }
    }
}

class Pivot(hardwareMap: HardwareMap) : System {
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")

    init {
        exHub.refreshBulkData()
        exHub.setJunkTicks()
    }

    override val dependencies: List<System> = emptyList()
    override val beforeRun = Command(PivotState.default(hardwareMap.getByName("pivot"), exHub.getEncoder(1)))
        .setOnEnter {
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.motor.power = 0.0

            it.pid.reset()
        }
        .setAction {
            exHub.refreshBulkData()
            println("${it.targetPosition}, ${exHub.getEncoderTicks(3).toDouble()}")
            val power = it.pid.calculate(it.targetPosition.toDouble(), exHub.getEncoderTicks(3).toDouble())
            it.motor.power = power

            false
        }
    override val afterRun = null

    //TODO: Swap to an angle
    var pivotTicks: Int
        set(value) {
            beforeRun.state.targetPosition = value
        }
        get() = beforeRun.state.targetPosition

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.leftTrigger.onHold {
            pivotTicks -= (500 * Scheduler.loopDeltaTime.seconds() * it).toInt()
        }

        gamepad.rightTrigger.onHold {
            pivotTicks += (500 * Scheduler.loopDeltaTime.seconds() * it).toInt()
        }
    }
}