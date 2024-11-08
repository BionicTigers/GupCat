package org.firstinspires.ftc.teamcode.mechanisms

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
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.motion.generateMotionProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Time
import org.firstinspires.ftc.teamcode.utils.assignTracker
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.sign

interface SlidesState : CommandState {
    var targetPosition: Int
    var profile: MotionResult
    val pid: PID
    val motor: DcMotorEx
    val junkTicks: Int
    var moveStartTime: Time
    var changed: Boolean

    companion object {
        fun default(name: String, motor: DcMotorEx): SlidesState {
            return object : SlidesState, CommandState by CommandState.default(name) {
                override var targetPosition = 0
                override val pid = PID(PIDTerms(2.0, 0.0), 0.0, 3800.0, -1.0, 1.0)
                override val motor = motor
                override var profile = generateMotionProfile(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                override val junkTicks = motor.currentPosition
                override var moveStartTime = Time()
                override var changed = false
            }
        }
    }
}

class Slides(hardwareMap: HardwareMap, val pivot: Pivot? = null) : System {
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
    val limitSwitch = hardwareMap.getByName<DigitalChannel>("slidesSwitch")

    override val dependencies = listOf(GamepadSystem.activeSystem!!) //TODO: make this not be so stupid (use a singleton)
    override val beforeRun = Command(SlidesState.default("Slides", hardwareMap.getByName("slides")))

        .setOnEnter {
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.motor.direction = DcMotorSimple.Direction.REVERSE
            it.motor.power = 0.0
            it.pid.reset()
            it.motor.assignTracker()
            exHub.refreshBulkData()
            exHub.setJunkTicks()
        }
        .setAction {
            exHub.refreshBulkData()
            val ticks = exHub.getEncoderTicks(2)

            println("$ticks, ${it.targetPosition}")
            val sub = 1400 - (pivot?.pivotTicks ?: 1400)
            targetPosition = targetPosition.coerceIn(-200, 50000 - sub  * 10)

            if (it.changed) {
//                it.profile = generateMotionProfile(it.motor.currentPosition, it.targetPosition, 40, 100, 400, it.motor.getTracker().velocity)
                it.moveStartTime = it.timeInScheduler
            }

            val deltaTime = it.timeInScheduler - it.moveStartTime
            val power = it.pid.calculate(it.targetPosition.toDouble(), ticks.toDouble())
            if (limitSwitch.state || it.targetPosition > 0)
                it.motor.power = power + .15 * sign(power)
            else
                it.motor.power = 0.0

            if (!limitSwitch.state)
                exHub.setJunkTicks()

            false
        }
    override val afterRun = null

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.getBooleanButton(Gamepad.Buttons.DPAD_UP).onHold {
            targetPosition += (25000 * Scheduler.loopDeltaTime.seconds()).toInt()
        }

        gamepad.getBooleanButton(Gamepad.Buttons.DPAD_DOWN).onHold {
            targetPosition -= (25000 * Scheduler.loopDeltaTime.seconds()).toInt()
        }
    }

    val currentPosition: Int
        get() = exHub.getEncoderTicks(2)

    var targetPosition = 0
        set(value) {
            val sub = 1400 - (pivot?.pivotTicks ?: 0)
            field = value.coerceIn(-200, 50000 - sub)
            beforeRun.state.changed = true
            beforeRun.state.targetPosition = value
        }
    var junkTicks = 0
}