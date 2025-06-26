package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.commands.BaseCommand
import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
import io.github.bionictigers.axiom.commands.persistentState
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.input.ControlSchema
import org.firstinspires.ftc.teamcode.input.Controllable
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.Gamepads
import org.firstinspires.ftc.teamcode.input.Profile
import org.firstinspires.ftc.teamcode.input.matches
import org.firstinspires.ftc.teamcode.input.types.Analog
import org.firstinspires.ftc.teamcode.input.types.Control
import org.firstinspires.ftc.teamcode.input.types.Digital
import org.firstinspires.ftc.teamcode.mechanisms.Slides.Companion.lowerProfile
import org.firstinspires.ftc.teamcode.mechanisms.Slides.Companion.raiseProfile
import org.firstinspires.ftc.teamcode.motion.MotionProfile
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Encoder
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds

class Pivot(hardwareMap: HardwareMap, telemetry: Telemetry? = null) : System, Controllable {
    companion object {
        /** Maximum ticks for the pivot encoder */
        const val MAX_TICKS = 1860

        /** Power applied when limit switch is active */
        const val RESTING_POWER = -0.2

        /** Maximum angle for the pivot */
        val MAX_ANGLE = Angle.degrees(90)

        /** Minimum angle for the pivot */
        val MIN_ANGLE = Angle.degrees(-5)

        /** Motion Profiling Values in degrees */
        val upProfile = MotionProfile(67200.0, 6723.39, 198.43, 13.4)

        /** Motion Profiling Values in degrees */
        val downProfile = MotionProfile(67200.8, 3493.54, 198.43, 13.4)
    }

    interface Schema : ControlSchema {
        /** Speed the pivot target angle changes per second */
        val rate: Angle

        /** Control to move the pivot upward */
        val up: Control<*>

        /** Control to move the pivot downward */
        val down: Control<*>

        /** Move to maximum position */
        val max: Digital?

        /** Move to resting position */
        val min: Digital?
    }

    override val name = "pivot"

    private val hub = ControlHub(hardwareMap, "Expansion Hub 2")

    private val dataState = DataState(hub.getEncoder(3), hardwareMap.getByName("pivotSwitch"))
    private val targetingState by persistentState("pivotTargeting") {
        TargetingState(
            hardwareMap.getByName("pivot"),
            hardwareMap.getByName("pivot2")
        )
    }

    val angle: Angle
        get() = dataState.angle

    private fun angleFromTicks(ticks: Int): Angle =
        Angle.degrees(ticks.toDouble() / MAX_TICKS * 90.0)

    fun moveTo(angle: Angle): Command<TargetingState> = Command.create("Pivot Move To", targetingState) {
        require(angle in MIN_ANGLE..MAX_ANGLE) { "Angle must be between $MIN_ANGLE and $MAX_ANGLE" }

        dependencies += beforeRun

        lateinit var motionResult: MotionResult

        enter {
            motionResult = if (angle > dataState.angle)
                upProfile.generate(dataState.angle.degrees, angle.degrees/*, dataState.velocity*/)
            else
                downProfile.generate(dataState.angle.degrees, angle.degrees/*, dataState.velocity*/)
        }

        action {
            //No need to coerce as it's done before power is applied
            it.targetAngle = Angle.degrees(motionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false))
            motionResult.getPosition(it.enteredAt!!.elapsedNow()) == motionResult.position.last()
        }
    }

    fun adjust(angle: Angle): Command<TargetingState> = Command.instant("Pivot Adjust", targetingState) {
        it.targetAngle = (it.targetAngle + angle).coerceIn(
            MIN_ANGLE,
            MAX_ANGLE
        )
    }

    fun min(): Command<TargetingState> = moveTo(Angle.ZERO)

    fun max(): Command<TargetingState> = moveTo(MAX_ANGLE)

    init {
        if (telemetry != null) {
            Scheduler.schedule(Command.continuous("Pivot Log") {
                telemetry.addData("Pivot Angle", angle.degrees)
                telemetry.addData("Pivot Target", targetingState.targetAngle.degrees)
                telemetry.addData("Pivot Resting", dataState.isResting)

//                telemetry.addData("Pivot minV", dataState.minVelocity)
//                telemetry.addData("Pivot maxV", dataState.maxVelocity)
//                telemetry.addData("Pivot minA", dataState.minAcceleration)
//                telemetry.addData("Pivot maxA", dataState.maxAcceleration)

            })
        }
    }

    override val beforeRun = Command.create("Pivot Data", dataState) {
        enter {
            it.encoder.refresh()
            it.encoder.setJunkTicks(Persistents.pivotTicks)
            it.angle = angleFromTicks(it.encoder.ticks)
        }

        action {
            it.isResting = !it.limitSwitch.state && targetingState.targetAngle <= Angle.degrees(0)

            if (it.isResting) {
                it.encoder.refresh()
                it.encoder.setJunkTicks()
                Persistents.pivotTicks = it.encoder.rawTicks
            }

            val lastAngle = it.angle
            val lastVelocity = it.velocity

            it.angle = angleFromTicks(it.encoder.ticks)
            it.velocity = (it.angle - lastAngle) / it.deltaTime.seconds
            it.maxVelocity = it.velocity
            it.minVelocity = it.velocity
            it.acceleration = (it.velocity - lastVelocity) / it.deltaTime.seconds
            it.maxAcceleration = it.acceleration
            it.minAcceleration = it.acceleration

            false
        }
    }

    override val afterRun = Command.create("Pivot Targeting", targetingState) {
        enter {
            it.targetAngle = Angle.ZERO

            it.motor1.apply {
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
                direction = DcMotorSimple.Direction.REVERSE
            }

            it.motor2.apply {
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
            }

            it.pid.reset()
        }

        action {
            val power = if (dataState.isResting) RESTING_POWER
            else it.pid.calculate(it.targetAngle.degrees, dataState.angle.degrees)

            it.motor1.power = power
            it.motor2.power = power

            false
        }
    }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.pivot) {
            if (!gamepad.matches(desiredGamepad)) return@with

            //Allow for smart casting
            when (val upControl = up) {
                is Digital -> builder.register(upControl) { adjust(rate * up.modifier * Scheduler.loopDeltaTime.seconds) }
                is Analog -> builder.register(upControl) { adjust(rate * it * up.modifier * Scheduler.loopDeltaTime.seconds) }
            }

            //Allow for smart casting
            when (val downControl = down) {
                is Digital -> builder.register(downControl) { adjust(-rate * up.modifier * Scheduler.loopDeltaTime.seconds) }
                is Analog -> builder.register(downControl) { adjust(-rate * it * up.modifier * Scheduler.loopDeltaTime.seconds) }
            }

            min?.let { builder.register(it) { min() } }
            max?.let { builder.register(it) { max() } }
        }

    data class DataState(
        val encoder: Encoder,
        val limitSwitch: DigitalChannel,
        var angle: Angle = Angle.radians(0),
        var velocity: Angle = Angle.radians(0),
        var acceleration: Angle = Angle.radians(0),
        var isResting: Boolean = false
    ) : BaseCommandState() {
        var maxVelocity = Angle.radians(0)
            set(value) {
                field = Angle.radians(field.radians.coerceAtLeast(value.radians))
            }
        var minVelocity = Angle.radians(0)
            set(value) {
                field = Angle.radians(field.radians.coerceAtMost(value.radians))
            }

        var maxAcceleration = Angle.radians(0)
            set(value) {
                field = Angle.radians(field.radians.coerceAtLeast(value.radians))
            }
        var minAcceleration = Angle.radians(0)
            set(value) {
                field = Angle.radians(field.radians.coerceAtMost(value.radians))
            }
    }

    data class TargetingState(
        val motor1: DcMotorEx,
        val motor2: DcMotorEx,
        @Editable
        val pid: PID = PID(
            PIDTerms(2.5, 50.0),
            0.0,
            MAX_ANGLE.degrees,
            -1.0,
            1.0
        ),
        var targetAngle: Angle = Angle.ZERO,
    ) : BaseCommandState()
}

/* Motion Profiling values
if (ticks > beforeRun.state.ticks) // up
            generateMotionProfile(beforeRun.state.ticks, ticks, 20000.0 / exHub.getVoltage() * 12.41 - slides.ticks / 7, 32000.9 / exHub.getVoltage() * 12.41, 3000.6 / exHub.getVoltage() * 12.41)
        else
            generateMotionProfile(beforeRun.state.ticks, ticks, 20000.0 / exHub.getVoltage() * 12.41 - slides.ticks / 7, 43981.9 / exHub.getVoltage() * 12.41, 2294.6 / exHub.getVoltage() * 12.41)
 */