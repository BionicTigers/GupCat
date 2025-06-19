package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Encoder
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.getByName

data class PivotDataState(
    val encoder: Encoder,
    val limitSwitch: DigitalChannel,
    var angle: Angle = Angle.radians(0),
    var velocity: Angle = Angle.radians(0),
    var acceleration: Angle = Angle.radians(0)
) : BaseCommandState("PivotData") {
    val isResting: Boolean
        get() = limitSwitch.state
}

data class PivotMoveState(
    val motor: DcMotorEx,
    val motor2: DcMotorEx,
    @Editable
    val pid: PID = PID(PIDTerms(0.0, 50.0), -0.0, 1860.0, -1.0, 1.0),
    var targetPosition: Angle = Angle.zero,
) : BaseCommandState("PivotTargeting")

class Pivot(hardwareMap: HardwareMap, val slides: Slides, val downLim: Double? = 0.0) : System {
    companion object {
        /** Maximum ticks for the pivot encoder */
        const val MAX_TICKS = 1860
        /** Power applied when limit switch is active */
        const val PIVOT_RESTING_POWER = -0.2
    }

    override val name = "pivot"
    override val dependencies: List<System> = emptyList()

    private val hub = ControlHub(hardwareMap, "Expansion Hub 2")

    private val pivotDataState = PivotDataState(hub.getEncoder(1), hardwareMap.getByName("pivotSwitch"))
    private val pivotTargetingState = Scheduler.getPersistentState("pivotTargeting", PivotMoveState(
        hardwareMap.getByName("pivot"),
        hardwareMap.getByName("pivot2")
    )) { targetPosition = Angle.zero }

    private fun calculateAngle(ticks: Int): Angle = Angle.degrees(ticks.toDouble() / MAX_TICKS * 90.0)

    private val gatherData = Command.create(pivotDataState) {
        onEnter {
            it.encoder.setJunkTicks(Persistents.pivotTicks)
            it.angle = calculateAngle(it.encoder.ticks)
        }

        action {
            if (it.isResting) {
                it.encoder.refresh()
                it.encoder.setJunkTicks()
                Persistents.pivotTicks = it.encoder.rawTicks
            }

            val lastAngle = it.angle
            val lastVelocity = it.velocity

            it.angle = calculateAngle(it.encoder.ticks)
            it.velocity = (it.angle - lastAngle) / it.deltaTime.seconds
            it.acceleration = (it.velocity - lastVelocity) / it.deltaTime.seconds

            false
        }
    }

    private val pivotTargeting = Command.create(pivotTargetingState) {
        onEnter {
            it.motor.apply {
                mode = DcMotor.RunMode.RUN_USING_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
                direction = DcMotorSimple.Direction.REVERSE
            }

            it.motor2.apply {
                mode = DcMotor.RunMode.RUN_USING_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
            }

            it.pid.reset()
        }

        action {
            val power = if (pivotDataState.isResting)
                it.pid.calculate(it.targetPosition.degrees, pivotDataState.angle.degrees)
            else
                PIVOT_RESTING_POWER

            it.motor.power = power
            it.motor2.power = power

            false
        }
    }

    override val beforeRun = gatherData
    override val afterRun = pivotTargeting
}