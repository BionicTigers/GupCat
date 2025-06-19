package org.firstinspires.ftc.teamcode.mechanisms.abc

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.commands.BaseCommandState
import org.firstinspires.ftc.robotcore.external.Telemetry
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
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
import kotlin.math.withSign

data class PivotState(
    val encoder: Encoder,
    val motor: DcMotorEx,
    val motor2: DcMotorEx,
    var targetPosition: Int = 0,
    @Editable
    val pid: PID = PID(PIDTerms(0.0, 50.0), -0.0, 1860.0, -1.0, 1.0),
    var ticks: Int = 0,
    var enabled: Boolean = true,
    var velocity: Double = 0.0,
    var acceleration: Double = 0.0,
    var moveStarted: Time? = null,
) : BaseCommandState()

class Pivot(hardwareMap: HardwareMap, val slides: Slides, val downLim: Double? = 0.0) : System {
    override val name = "pivot"
    override val dependencies: List<System> = emptyList()

    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
    val limitSwitch = hardwareMap.getByName<DigitalChannel>("pivotSwitch")

    val offset = interpolatedMapOf(
        0.0 to 0.0,
        50.0 to 0.0,
        400.0 to 0.2,
        750.0 to 0.25,
        1500.0 to 0.18
    )

    val ticks: Int
        get() = beforeRun.state.ticks
    val pivotAngle: Angle
        get() = Angle.degrees(ticks / max.toDouble() * 90)

    var switchPressed = true

    var motor1Pow = 0.0
    var motor2Pow = 0.0

    var oldVel = 0.0
    var oldAccel = 0.0

    var motionProfile: MotionResult? = null

    var weird = false

    override val beforeRun = Command(PivotState.default(hardwareMap.getByName("pivot"), hardwareMap.getByName("pivot2"), exHub.getEncoder(1)))
        .setOnEnter {
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.motor.power = 0.0
            it.motor.direction = DcMotorSimple.Direction.REVERSE
            it.motor2.power = 0.0
//            it.motor2.direction = DcMotorSimple.Direction.REVERSE
            if (downLim != 0.0)
                it.pid.pvMin = downLim!!

            it.pid.reset()

//            println(Persistents.pivotTicks)
//            exHub.setEncoderDirection(3, ControlHub.Direction.Backward)
            if (Persistents.pivotTicks == null) Persistents.pivotTicks = exHub.rawGetEncoderTicks(3)
            println("TICKS PERSISTENT START ${Persistents.pivotTicks}")
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

            if (it.targetPosition > ticks && ticks < 235) {
                it.pid.kP = 3.0
            } else {
                it.pid.kP = 2.5
            }

            var pidPower = it.pid.calculate(it.targetPosition.toDouble(), ticks.toDouble())

            val power = pidPower + ((slides.ticks / slides.max) * .05 * (1 - ticks / max)).withSign(it.targetPosition - ticks)

            if (limitSwitch.state || it.targetPosition > 0) {
                switchPressed = false

                it.motor.power = power
                it.motor2.power = power

                it.pid.tI = 50.0
            } else {
                switchPressed = true

                it.motor2.power = -.2
                it.motor.power = -.2

                it.pid.tI = 0.0
            }
            // 600

            if (!limitSwitch.state && exHub.rawGetEncoderTicks(3) != 0) {
                exHub.setJunkTicks()
                Persistents.pivotTicks = exHub.rawGetEncoderTicks(3)
                println("TICKS PERSISTENT ${Persistents.pivotTicks}")
            }

//            if (weird) {
//                it.motor2.power = -.4
//                it.motor.power = -.4
//            }

//            println(Persistents.pivotTicks)

//            telemetry.addData("Pivot Process P-VAL", it.pid.kP)
//            telemetry.addData("Pivot Process Value", exHub.getEncoderTicks(3))
//            telemetry.addData("Pivot Set Point", it.targetPosition)
//            telemetry.update()
            motor1Pow = it.motor.power
            motor2Pow = it.motor2.power
            false
        }
    override val afterRun = null

    val max = 1860

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
        gamepad.getBooleanButton(Gamepad.Buttons.RIGHT_STICK_BUTTON).onDown {
            pivotTicks = 600
        }
    }

//    fun setupWeirdDC(gamepad: Gamepad) {
//        gamepad.leftTrigger.onHold {
//            if (limitSwitch.state) {
//                weird = true
//            } else {
//                weird = false
//            }
//        }
//        gamepad.leftTrigger.onUp {
//            weird = false
//        }
//    }
//
//    fun powerDown() {
//
//    }

    var maxVelocity = 0.0
    var maxAccel = 0.0

    fun log(telemetry: Telemetry) {
        maxVelocity = max(abs(beforeRun.state.velocity), maxVelocity)
        maxAccel = max(abs(beforeRun.state.acceleration), maxAccel)

        telemetry.addData("power 1", motor1Pow)
        telemetry.addData("power 2", motor2Pow)
        telemetry.addData("switch pressed",switchPressed)
        telemetry.addData("pivotTicks", pivotTicks)
        telemetry.addData("pivotActualTicks", ticks)
        telemetry.addData("max velocity" , maxVelocity)
        telemetry.addData("max acceleration" , maxAccel)
    }
}