package org.firstinspires.ftc.teamcode.mechanisms

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
import io.github.bionictigers.axiom.utils.Time
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.teamcode.input.Gamepad
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.motion.generateMotionProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Encoder
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.interpolatedMapOf
import kotlin.math.abs
import kotlin.math.max

interface PivotState : CommandState {
    val encoder: Encoder
    var targetPosition: Int
    val pid: PID
    var ticks: Int
    val motor: DcMotorEx
    val motor2: DcMotorEx
    var enabled: Boolean
    var velocity: Double
    var acceleration: Double
    var moveStarted: Time?

    companion object {
        fun default(motor: DcMotorEx, motor2: DcMotorEx, encoder: Encoder): PivotState {
            return object : PivotState, CommandState by CommandState.default("Pivot") {
                override val encoder = encoder
                override var targetPosition = 0
                @Editable
                override val pid = PID(PIDTerms(3.0, 40.0), 0.0, 2040.0, -1.0, 1.0)
                override var ticks = 0
                override val motor = motor
                override val motor2 = motor2
                override var enabled = true
                override var velocity = 0.0
                override var acceleration = 0.0
                override var moveStarted: Time? = null
            }
        }
    }
}

class Pivot(hardwareMap: HardwareMap, val slides: Slides) : System {
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
    val limitSwitch = hardwareMap.getByName<DigitalChannel>("pivotSwitch")

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
        50.0 to 0.0,
        400.0 to 0.2,
        750.0 to 0.25,
        1500.0 to 0.18
    )

    val ticks: Int
        get() = beforeRun.state.ticks
    var enabled: Boolean
        get() = beforeRun.state.enabled
        set(value) { beforeRun.state.enabled = value }
    val pivotAngle: Angle
        get() = Angle.degrees(ticks / max.toDouble() * 90)

    var oldVel = 0.0
    var oldAccel = 0.0

    var motionProfile: MotionResult? = null

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
//            exHub.setEncoderDirection(3, ControlHub.Direction.Backward)
            if (Persistents.pivotTicks == null) Persistents.pivotTicks = exHub.rawGetEncoderTicks(3)
            exHub.setJunkTicks(3, Persistents.pivotTicks)
        }
        .setAction {
            if (!enabled) {
                it.motor.power = 0.0
                it.motor2.power = 0.0
                return@setAction false
            }

            val oldTicks = it.ticks
            oldVel = it.velocity
            oldAccel = it.acceleration

            exHub.refreshBulkData()
            it.ticks = exHub.getEncoderTicks(3)

            it.velocity = (it.ticks - oldTicks) / it.deltaTime.seconds()
            if (abs(oldVel) < abs(it.velocity))
                it.acceleration = (it.velocity - oldVel) / it.deltaTime.seconds()

//            if (it.targetPosition >= ticks)
//                it.pid.kP = upPIDTerms[ticks.toDouble()]
//            else
//                it.pid.kP = downPIDTerms[ticks.toDouble()]

            if (motionProfile != null) {
                it.targetPosition =
                    motionProfile!!.getPosition(it.timeInScheduler - it.moveStarted!!).toInt()
            }

            var pidPower = it.pid.calculate(it.targetPosition.toDouble(), ticks.toDouble())

            val power = pidPower + (slides.ticks / slides.max) * .05 * (1 - ticks / max)
            if (limitSwitch.state || it.targetPosition > 0) {
                it.motor.power = power
                it.motor2.power = power
            } else {
                it.motor2.power = -.15
                it.motor.power = -.15
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

    val max = 2040

    //TODO: Swap to an angle
    var pivotTicks: Int
        set(value) {
            beforeRun.state.targetPosition = value.coerceIn(-100,max)
        }
        get() = beforeRun.state.targetPosition

    val currentPosition: Int
        get() = exHub.getEncoderTicks(3)

    var previous = 0
    fun mpSetPosition(ticks: Int) {
        if (previous == ticks) return
        previous = ticks
        motionProfile = if (ticks > beforeRun.state.ticks) // up
            generateMotionProfile(beforeRun.state.ticks, ticks, 20000.0 / exHub.getVoltage() * 12.41 - slides.ticks / 7, 32000.9 / exHub.getVoltage() * 12.41, 3000.6 / exHub.getVoltage() * 12.41)
        else
            generateMotionProfile(beforeRun.state.ticks, ticks, 20000.0 / exHub.getVoltage() * 12.41 - slides.ticks / 7, 43981.9 / exHub.getVoltage() * 12.41, 2294.6 / exHub.getVoltage() * 12.41)
        beforeRun.state.moveStarted = beforeRun.state.timeInScheduler
    }

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.leftTrigger.onHold {
            pivotTicks -= (1500 * Scheduler.loopDeltaTime.seconds() * it).toInt()
            motionProfile = null
            beforeRun.state.moveStarted = null
        }

        gamepad.rightTrigger.onHold {
            pivotTicks += (1500 * Scheduler.loopDeltaTime.seconds() * it).toInt()
            motionProfile = null
            beforeRun.state.moveStarted = null
        }
    }

    var maxVelocity = 0.0
    var maxAccel = 0.0

    fun log(telemetry: Telemetry) {
        maxVelocity = max(abs(beforeRun.state.velocity), maxVelocity)
        maxAccel = max(abs(beforeRun.state.acceleration), maxAccel)

        telemetry.addData("pivotTicks", pivotTicks)
        telemetry.addData("pivotActualTicks", ticks)
        telemetry.addData("max velocity" , maxVelocity)
        telemetry.addData("max acceleration" , maxAccel)
    }
}