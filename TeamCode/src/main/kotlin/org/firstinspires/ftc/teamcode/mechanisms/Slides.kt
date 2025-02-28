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
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.motion.generateMotionProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Persistents
import io.github.bionictigers.axiom.utils.Time
import org.firstinspires.ftc.teamcode.utils.assignTracker
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.max
import kotlin.math.min

object SlidesPID {
    @JvmField
    var p = 2.0
    @JvmField
    var i = 100.0
    @JvmField
    var d = 0.0
}

interface SlidesState : CommandState {
    var targetPosition: Int
    var profile: MotionResult
    val pid: PID
    val motorL: DcMotorEx
    val motorR: DcMotorEx
    var moveStartTime: Time
    var changed: Boolean
    var limitSwitch: DigitalChannel
    var processValue: Int
    var setPoint: Int

    companion object {
        fun default(name: String, motorL: DcMotorEx, motorR: DcMotorEx, limitSwitch: DigitalChannel): SlidesState {
            return object : SlidesState, CommandState by CommandState.default(name) {
                override var targetPosition = 0
                override val pid = PID(PIDTerms(SlidesPID.p, SlidesPID.i, SlidesPID.d), 0.0, 53500.0, -1.0, 1.0)
                override val motorL = motorL
                override val motorR = motorR
                override var profile = generateMotionProfile(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                override var moveStartTime = Time()
                override var changed = false
                override var limitSwitch = limitSwitch
                override var processValue = 0
                override var setPoint = 0
            }
        }
    }
}

class Slides(hardwareMap: HardwareMap, pivot: Pivot) : System {
    private val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
    private val max = 53500
    private val pivotDownMax = 26500
    private val pivotMaxVelocity = 50000

    var lastTicks = 0
    var ticks = 0
    var velocityMax = 0.0
    var lastVelocity = 0.0
    var acceleration = 0.0
    var accelerationMax = 0.0

    override val dependencies = listOf(GamepadSystem.activeSystem!!) //TODO: make this not be so stupid (use a singleton)
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
            ticks = exHub.getEncoderTicks(2)

            val velocity = (ticks - lastTicks) / it.deltaTime.seconds()
            lastTicks = ticks
            velocityMax = max(velocityMax, velocity)
            acceleration = (velocity - lastVelocity) / it.deltaTime.seconds()
            accelerationMax = max(accelerationMax, acceleration)
            lastVelocity = velocity
            val percent = min(pivot.pivotTicks, pivot.max) / pivot.max
            it.targetPosition = targetPosition.coerceIn(-200, max + percent * (max - pivotDownMax))

//            if (it.changed && it.timeInScheduler - it.moveStartTime > Time.fromSeconds(.5)) {
//                it.profile = generateMotionProfile(ticks, it.targetPosition, 60000, 3000000, 50000)
//                it.moveStartTime = it.timeInScheduler
//                it.changed = false
//            }

            val direction = if (it.targetPosition >= ticks.toDouble()) -1 else 1
            if (it.targetPosition >= ticks.toDouble()) {
                it.pid.kP = 12.0
            } else {
                it.pid.kP = 9.0
            }

            val power = if (it.limitSwitch.state || it.targetPosition > 0) {
//                it.pid.calculate(it.profile.getPosition(it.timeInScheduler - it.moveStartTime), ticks.toDouble())
                it.pid.calculate(it.targetPosition.toDouble(), ticks.toDouble())
            } else {
                -0.02
            }

//            println("Target: ${it.profile.getPosition(it.timeInScheduler - it.moveStartTime)}, Time: ${it.timeInScheduler - it.moveStartTime}, Actual: ${it.targetPosition}")

            if (!it.limitSwitch.state) {
                ticks = 0
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
        }

        gamepad.getBooleanButton(Gamepad.Buttons.DPAD_DOWN).onHold {
            targetPosition -= (max * .8 * Scheduler.loopDeltaTime.seconds()).toInt()
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

    fun log(telemetry: Telemetry) {
        telemetry.addData("SlidesCurrent", ticks)
        telemetry.addData("SlidesTarget", targetPosition)
        telemetry.addData("SlideMaxVelocity", velocityMax)
        telemetry.addData("SlideMaxAcceleration", accelerationMax)
    }
}