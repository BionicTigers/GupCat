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

class Slides(hardwareMap: HardwareMap, private val pivot: Pivot? = null, telemetry: Telemetry? = null) : System, Controllable {
    companion object {
        /** Max Ticks for the slide encoder */
        const val MAX_TICKS = 52500

        /** Max Ticks for the slide encoder */
        const val MIN_TICKS = -3000

        /** Max Ticks for the slide encoder */
        const val PIVOT_RESTING_MAX_TICKS = 26500

        /** Power applied when limit switch is active */
        const val RESTING_POWER = -0.02

        /** Motion profile for raising the slides */
        val raiseProfile = MotionProfile(30000000, 2925264, 54886.41, 12.57)

        /** Motion profile for lowering the slides */
        val lowerProfile = MotionProfile(17000000, 1656291, 88136, 12.57)
    }

    interface Schema : ControlSchema {
        /** Speed the slides target ticks changes per second */
        val rate: Int

        /** Control to move the slides upward */
        val raise: Control<*>

        /** Control to move the slides downward */
        val lower: Control<*>

        /** Move to maximum position */
        val max: Digital?

        /** Move to minimum position */
        val min: Digital?
    }

    override val name = "Slides"

    private val hub = ControlHub(hardwareMap, "Expansion Hub 2")

    private val dataState = DataState(hub.getEncoder(2), hardwareMap.getByName("slideLimit"))
    private val targetingState by persistentState("slidesTargeting") {
        TargetingState(
            hardwareMap.getByName("slidesL"),
            hardwareMap.getByName("slidesR")
        )
    }

    /**
     * Limits max ticks due to horizontal expansion limit
     *
     * Assumes no expansion limit if pivot not present
     */
    val adjustedMaxTicks: Int
        get() {
            val slope = (MAX_TICKS - PIVOT_RESTING_MAX_TICKS).toDouble()
            val pivotPercentFromMax =
                pivot?.let { it.angle.degrees / Pivot.MAX_ANGLE.degrees } ?: 1.0
//            println("s: $slope, p: ${pivotPercentFromMax * 100}, c: ${slope * pivotPercentFromMax}")
            return (PIVOT_RESTING_MAX_TICKS + slope * pivotPercentFromMax).toInt()
        }

    /** Current ticks of the slide encoder */
    val ticks: Int
        get() = dataState.ticks

    fun moveTo(ticks: Number): Command<TargetingState> = Command.create("Slides Move To", targetingState) {
        require(ticks.toInt() in MIN_TICKS..MAX_TICKS) { "Ticks must be between $MIN_TICKS and $adjustedMaxTicks" }

        dependencies += beforeRun

        lateinit var motionResult: MotionResult

        enter {
            motionResult = if (ticks.toInt() > dataState.ticks)
                raiseProfile.generate(dataState.ticks, ticks/*, dataState.velocity*/)
            else
                lowerProfile.generate(dataState.ticks, ticks/*, dataState.velocity*/)
        }

        action {
            //No need to coerce as it's done before power is applied
            it.targetTicks = motionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false).toInt()
            it.targetTicks == motionResult.position.last().toInt()
        }
    }

    fun adjust(ticks: Number): Command<TargetingState> = Command.instant("Slides Adjust", targetingState) {
        //No need to coerce as it's done before power is applied
        targetingState.targetTicks += ticks.toInt()
    }

    fun min(): Command<TargetingState> = moveTo(MIN_TICKS)

    fun max(): Command<TargetingState> = moveTo(MAX_TICKS)

    init {
        if (telemetry != null) {
            Scheduler.schedule(Command.continuous("Slides Log") {
                telemetry.addData("Slide Ticks", dataState.ticks)
                telemetry.addData("Slide Target", targetingState.targetTicks)
                telemetry.addData("Slide Resting", dataState.isResting)

                telemetry.addData("Slide minV", dataState.minVelocity)
                telemetry.addData("Slide maxV", dataState.maxVelocity)
                telemetry.addData("Slide minA", dataState.minAcceleration)
                telemetry.addData("Slide maxA", dataState.maxAcceleration)
            })
        }
    }

    override val beforeRun = Command.create("Slides Data", dataState) {
        enter {
            it.encoder.refresh()
            it.encoder.setJunkTicks(Persistents.slideTicks)
            it.ticks = it.encoder.ticks
        }

        action {
            it.isResting = !it.limitSwitch.state && targetingState.targetTicks <= 0

            if (it.isResting) {
                it.encoder.refresh()
                it.encoder.setJunkTicks()
                Persistents.slideTicks = it.encoder.rawTicks
            }

            val lastTicks = it.ticks
            val lastVelocity = it.velocity

            it.ticks = it.encoder.ticks
            it.velocity = (it.ticks - lastTicks) / it.deltaTime.seconds
            it.minVelocity = it.velocity
            it.maxVelocity = it.velocity
            it.acceleration = (it.velocity - lastVelocity) / it.deltaTime.seconds
            it.minAcceleration = it.acceleration
            it.maxAcceleration = it.acceleration

            false
        }
    }

    override val afterRun = Command.create("Slides Targeting", targetingState) {
        enter {
            it.motorL.apply {
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
            }

            it.motorR.apply {
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                direction = DcMotorSimple.Direction.REVERSE
                power = 0.0
            }

            it.manualPid.reset()
        }

        action {
            it.targetTicks = it.targetTicks.coerceIn(MIN_TICKS, adjustedMaxTicks)

            val power = if (dataState.isResting) RESTING_POWER
            else it.manualPid.calculate(it.targetTicks.toDouble(), dataState.ticks.toDouble())

            it.motorL.power = power
            it.motorR.power = power

            false
        }
    }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.slides) {
            if (!gamepad.matches(desiredGamepad)) return@with

            //Allow for smart casting
            when (val raiseControl = raise) {
                is Digital -> builder.register(raiseControl) { adjust(rate * raise.modifier * Scheduler.loopDeltaTime.seconds) }
                is Analog -> builder.register(raiseControl) { adjust(rate * it * raise.modifier * Scheduler.loopDeltaTime.seconds) }
            }

            //Allow for smart casting
            when (val lowerControl = lower) {
                is Digital -> builder.register(lowerControl) { adjust(-rate * lower.modifier * Scheduler.loopDeltaTime.seconds) }
                is Analog -> builder.register(lowerControl) { adjust(-rate * it * lower.modifier * Scheduler.loopDeltaTime.seconds) }
            }

            min?.let { builder.register(it) { min() } }
            max?.let { builder.register(it) { max() } }
        }


    data class DataState(
        val encoder: Encoder,
        val limitSwitch: DigitalChannel,
        var ticks: Int = 0,
        var velocity: Double = 0.0,
        var acceleration: Double = 0.0,
        var isResting: Boolean = false
    ) : BaseCommandState() {
        var maxVelocity = 0.0
            set(value) {
                field = field.coerceAtLeast(value)
            }
        var minVelocity = 0.0
            set(value) {
                field = field.coerceAtMost(value)
            }

        var maxAcceleration = 0.0
            set(value) {
                field = field.coerceAtLeast(value)
            }
        var minAcceleration = 0.0
            set(value) {
                field = field.coerceAtMost(value)
            }
    }

    data class TargetingState(
        val motorL: DcMotorEx,
        val motorR: DcMotorEx,
        var targetTicks: Int = 0,
        @Editable
        val manualPid: PID = PID(PIDTerms(7.0, 50.0, 0.0), 0.0, MAX_TICKS.toDouble(), -1.0, 1.0),
        @Editable
        val motionPid: PID = PID(PIDTerms(18.0, 30.0, 0.0), 0.0, MAX_TICKS.toDouble(), -1.0, 1.0)
    ) : BaseCommandState()
}