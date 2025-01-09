package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.System
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.motion.generateMotionProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Time
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.assignTracker
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.sign

interface SlidesState : CommandState {
    var targetPosition: Int
    var profile: MotionResult
    val pid: PID
    val motorL: DcMotorEx
    val motorR: DcMotorEx
    var moveStartTime: Time
    var changed: Boolean
    var limitSwitch: DigitalChannel

    companion object {
        fun default(name: String, motorL: DcMotorEx, motorR: DcMotorEx, limitSwitch: DigitalChannel): SlidesState {
            return object : SlidesState, CommandState by CommandState.default(name) {
                override var targetPosition = 0
                override val pid = PID(PIDTerms(2.0, 100.0), 0.0, 3050.0, -1.0, 1.0)
                override val motorL = motorL
                override val motorR = motorR
                override var profile = generateMotionProfile(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                override var moveStartTime = Time()
                override var changed = false
                override var limitSwitch = limitSwitch
            }
        }
    }
}

class Slides(hardwareMap: HardwareMap, val pivot: Pivot) : System {
    private val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
    private val max = 3500
    private val pivotDownMax = 2300

//    val velocity = Vector2((deltaLocalX + deltaStrafeX).mm / deltaTime, (deltaLocalY + deltaStrafeY).mm / deltaTime)

    override val dependencies = listOf(GamepadSystem.activeSystem!!) //TODO: make this not be so stupid (use a singleton)
    override val beforeRun = Command(SlidesState.default("Slides", hardwareMap.getByName("slidesL"), hardwareMap.getByName("slidesR"), hardwareMap.getByName("slideLimit")))
        .setOnEnter {
            it.motorR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motorR.power = 0.0
            it.motorL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motorL.direction = DcMotorSimple.Direction.REVERSE
            it.motorL.power = 0.0
            it.pid.reset()
            it.motorR.assignTracker()
            exHub.refreshBulkData()
            exHub.setJunkTicks()
        }
        .setAction {
            exHub.refreshBulkData()
            ticks = exHub.getEncoderTicks(2)

            val percent = 1 - pivot.pivotTicks / pivot.max
            targetPosition = targetPosition.coerceIn(-200, max - percent * (max - pivotDownMax))

//            if (it.changed) {
//                it.profile = generateMotionProfile(ticks, it.targetPosition, 40, 100, 400, it.motor.getTracker().velocity)
//                it.moveStartTime = it.timeInScheduler
//            }

            val direction = if (it.targetPosition >= ticks.toDouble()) -1 else 1
            val deltaTime = it.timeInScheduler - it.moveStartTime
            if (it.targetPosition >= ticks.toDouble()) {
                it.pid.kP = 12.0
            } else {
                it.pid.kP = 9.0
            }


            val power: Double
//            if (it.limitSwitch.state || it.targetPosition > 0) {
                power = it.pid.calculate(it.targetPosition.toDouble(), ticks.toDouble())
//            } else {
//                power = -0.02
//            }

            it.motorR.power = power //+ .15 * power * direction
            it.motorL.power = power

            false
        }
    override val afterRun = null

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.getBooleanButton(Gamepad.Buttons.DPAD_UP).onHold {
            targetPosition += (2400 * Scheduler.loopDeltaTime.seconds()).toInt()
        }

        gamepad.getBooleanButton(Gamepad.Buttons.DPAD_DOWN).onHold {
            targetPosition -= (2400 * Scheduler.loopDeltaTime.seconds()).toInt()
        }
    }

    var targetPosition = 0
        set(value) {
//            val sub = 400 - (pivot?.pivotTicks ?: 400)
            field = value.coerceIn(-200, max)
            beforeRun.state.changed = true
            beforeRun.state.targetPosition = value
        }
    var ticks = 0

    fun log(telemetry: Telemetry) {
        telemetry.addData("SlidesCurrent", ticks)
        telemetry.addData("SlidesTarget", targetPosition)
    }
}