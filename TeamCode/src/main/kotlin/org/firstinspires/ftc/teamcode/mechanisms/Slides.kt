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
import org.firstinspires.ftc.teamcode.input.Gamepad
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.motion.generateMotionProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Persistents
import io.github.bionictigers.axiom.utils.Time
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.withSign

interface SlidesState : CommandState {
    var targetPosition: Int
    var profile: MotionResult?
    val pid: PID
    val gpPid: PID
    val motorL: DcMotorEx
    val motorR: DcMotorEx
    var moveStartTime: Time?
    var changed: Boolean
    var limitSwitch: DigitalChannel
    var ticks: Int
    var setPoint: Int

    companion object {
        fun default(name: String, motorL: DcMotorEx, motorR: DcMotorEx, limitSwitch: DigitalChannel): SlidesState {
            return object : SlidesState, CommandState by CommandState.default(name) {
                @Editable
                override var targetPosition = 0
                @Editable
                override val pid = PID(PIDTerms(0.0, 30.0, 0.0), 0.0, 52500.0, -1.0, 1.0)
                override val gpPid = PID(PIDTerms(0.0,50.0,0.0), 0.0, 52500.0, -1.0, 1.0)
                override val motorL = motorL
                override val motorR = motorR
                override var profile: MotionResult? = null
                override var moveStartTime: Time? = null
                override var changed = false
                override var limitSwitch = limitSwitch
                override var ticks = 0
                override var setPoint = 0
            }
        }
    }
}

class Slides(hardwareMap: HardwareMap, var pivot: Pivot? = null) : System {
    private val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
    val max = 52500
    private val pivotDownMax = 26500
    private val pivotMaxVelocity = 50000

    var lastTicks = 0
    val ticks: Int
        get() = beforeRun.state.ticks
    var velocityMax = 0.0
    var lastVelocity = 0.0
    var acceleration = 0.0
    var accelerationMax = 0.0

    override val dependencies: List<System> = emptyList() //TODO: make this not be so stupid (use a singleton)
    override val beforeRun = Command(SlidesState.default("Slides", hardwareMap.getByName("slidesL"), hardwareMap.getByName("slidesR"), hardwareMap.getByName("slideLimit")))
        .setOnEnter {
            it.motorR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motorR.direction = DcMotorSimple.Direction.REVERSE
            it.motorR.power = 0.0
            it.motorL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motorL.power = 0.0
            it.pid.reset()
            exHub.refreshBulkData()
            if (Persistents.slideTicks == null) Persistents.slideTicks = exHub.rawGetEncoderTicks(2)
            exHub.setJunkTicks(2, Persistents.slideTicks)
        }
        .setAction {
            exHub.refreshBulkData()
            it.ticks = exHub.getEncoderTicks(2)

            val velocity = (ticks - lastTicks) / it.deltaTime.seconds()
            lastTicks = ticks
            velocityMax = max(velocityMax, abs(velocity))

            if (abs(lastVelocity) < abs(velocity))
                acceleration = (velocity - lastVelocity) / it.deltaTime.seconds()

            accelerationMax = max(accelerationMax, abs(acceleration))
            lastVelocity = velocity

            if (it.profile != null) {
                it.targetPosition =
                    it.profile!!.getPosition(it.timeInScheduler - it.moveStartTime!!).toInt()
            }

            val slope = if (pivot != null) (max - pivotDownMax) / pivot!!.max else 0
            val ticksFrom90 = if (pivot != null) pivot!!.max - pivot!!.ticks else 0
            it.targetPosition = targetPosition.coerceIn(-200, max - slope * ticksFrom90)

            val direction = if (it.targetPosition >= ticks.toDouble()) -1 else 1

            if (it.targetPosition >= ticks.toDouble()) {
                it.pid.kP = 18.0
                it.gpPid.kP = 14.0
            } else {
                it.pid.kP = 14.0
                it.gpPid.kP = 10.0
            }

            val power = if (it.limitSwitch.state || it.targetPosition > 0) {
//                it.pid.calculate(it.profile.getPosition(it.timeInScheduler - it.moveStartTime), ticks.toDouble())
                if (it.profile != null) {
                    it.pid.calculate(it.targetPosition.toDouble(), ticks.toDouble())
                } else {
                    it.gpPid.calculate(it.targetPosition.toDouble(), ticks.toDouble())
                } + .1.withSign(it.targetPosition - ticks)
            } else {
                -0.02
            }

            println(power)

//            println("Target: ${it.profile.getPosition(it.timeInScheduler - it.moveStartTime)}, Time: ${it.timeInScheduler - it.moveStartTime}, Actual: ${it.targetPosition}")

            if (!it.limitSwitch.state) {
                it.ticks = 0
                Persistents.slideTicks = exHub.rawGetEncoderTicks(2)
                exHub.setJunkTicks(2, Persistents.slideTicks)
            }

//            println(it.limitSwitch.state)

            it.motorR.power = power //+ .15 * power * direction
            it.motorL.power = power

//            it.setPoint = it.profile.getPosition(it.timeInScheduler - it.moveStartTime).toInt()
//            it.processValue = ticks
//            println(Persistents.slideTicks)

            false
        }
    override val afterRun = null

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.getBooleanButton(Gamepad.Buttons.DPAD_UP).onHold {
            targetPosition += (max * .8 * Scheduler.loopDeltaTime.seconds()).toInt()
            beforeRun.state.profile = null
            beforeRun.state.moveStartTime = null
        }

        gamepad.getBooleanButton(Gamepad.Buttons.DPAD_DOWN).onHold {
            targetPosition -= (max * .8 * Scheduler.loopDeltaTime.seconds()).toInt()
            beforeRun.state.profile = null
            beforeRun.state.moveStartTime = null
        }

        gamepad.leftJoystick.continuous {
            targetPosition -= (max * .35 * Scheduler.loopDeltaTime.seconds() * it.y * 1.1).toInt()
            beforeRun.state.profile = null
            beforeRun.state.moveStartTime = null
        }
    }

    var targetPosition: Int = 0
        get() = beforeRun.state.targetPosition
        set(value) {
//            val sub = 400 - (pivot?.pivotTicks ?: 400)
            field = value.coerceIn(-2000, max)
            beforeRun.state.changed = true
            beforeRun.state.targetPosition = value
        }

    var previous = 0
    fun mpMove(ticks: Int) {
        if (previous == ticks) return
        previous = ticks
        if (ticks > beforeRun.state.ticks)
            beforeRun.state.profile = generateMotionProfile(beforeRun.state.ticks, ticks, 300000, 1120130, 51000)
        else
            beforeRun.state.profile = generateMotionProfile(beforeRun.state.ticks, ticks, 300000, 3692851, 59948)
        beforeRun.state.moveStartTime = beforeRun.state.timeInScheduler
    }

    fun log(telemetry: Telemetry) {
        telemetry.addData("SlidesCurrent", ticks)
        telemetry.addData("SlidesTarget", targetPosition)
        telemetry.addData("SlideMaxVelocity", velocityMax)
        telemetry.addData("SlideMaxAcceleration", accelerationMax)
    }
}