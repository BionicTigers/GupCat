package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.InstantCommand
import io.github.bionictigers.axiom.commands.System
import io.github.bionictigers.axiom.commands.persistentState
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.teamcode.input.ControlSchema
import org.firstinspires.ftc.teamcode.input.Controllable
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.Gamepads
import org.firstinspires.ftc.teamcode.input.Profile
import org.firstinspires.ftc.teamcode.input.matches
import org.firstinspires.ftc.teamcode.input.types.Analog
import org.firstinspires.ftc.teamcode.input.types.Control
import org.firstinspires.ftc.teamcode.input.types.Digital
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Encoder
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds

class Slides(hardwareMap: HardwareMap, val pivot: Pivot? = null) : System, Controllable {
    companion object {
        /** Max Ticks for the slide encoder */
        const val MAX_TICKS = 47000

        /** Max Ticks for the slide encoder */
        const val MIN_TICKS = -3000

        /** Max Ticks for the slide encoder */
        const val PIVOT_RESTING_MAX_TICKS = 26500

        /** Power applied when limit switch is active */
        const val RESTING_POWER = -0.02
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
            val slope = (MAX_TICKS - PIVOT_RESTING_MAX_TICKS).toDouble() / Pivot.MAX_ANGLE.degrees
            val pivotPercentFromMax =
                pivot?.let { it.angle.degrees / Pivot.MAX_ANGLE.degrees } ?: 1.0
            return (PIVOT_RESTING_MAX_TICKS + slope * pivotPercentFromMax).toInt()
        }

    //TODO: Change this to use motion profiling
    fun moveTo(ticks: Number): Command<BaseCommandState> = InstantCommand {
        //No need to coerce as it's done before power is applied
        targetingState.targetTicks = ticks.toInt()
    }

    fun adjust(ticks: Number): Command<BaseCommandState> = InstantCommand {
        //No need to coerce as it's done before power is applied
        targetingState.targetTicks += ticks.toInt()
    }

    fun min(): Command<BaseCommandState> = moveTo(MIN_TICKS)

    fun max(): Command<BaseCommandState> = moveTo(adjustedMaxTicks)

    override val beforeRun = Command.create("SlidesData", dataState) {
        enter {
            it.encoder.refresh()
            it.encoder.setJunkTicks(Persistents.slideTicks)
            it.ticks = it.encoder.ticks
        }

        action {
            if (it.isResting) {
                it.encoder.refresh()
                it.encoder.setJunkTicks()
                Persistents.slideTicks = it.encoder.rawTicks
            }

            val lastTicks = it.ticks
            val lastVelocity = it.velocity

            it.ticks = it.encoder.ticks
            it.velocity = (it.ticks - lastTicks) / it.deltaTime.seconds
            it.acceleration = (it.velocity - lastVelocity) / it.deltaTime.seconds

            false
        }
    }

    override val afterRun = Command.create("SlidesTargeting", targetingState) {
        enter {
            it.motorL.apply {
                mode = DcMotor.RunMode.RUN_USING_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
            }

            it.motorR.apply {
                mode = DcMotor.RunMode.RUN_USING_ENCODER
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
            val raiseControl = raise
            when (raiseControl) {
                is Digital -> builder.register(raiseControl) { adjust(rate * raise.modifier) }
                is Analog -> builder.register(raiseControl) { adjust(rate * it * raise.modifier) }
            }

            //Allow for smart casting
            val lowerControl = lower
            when (lowerControl) {
                is Digital -> builder.register(lowerControl) { adjust(-rate * lower.modifier) }
                is Analog -> builder.register(lowerControl) { adjust(-rate * it * lower.modifier) }
            }

            min?.let { builder.register(it) { min() } }
            max?.let { builder.register(it) { max() } }
        }


    data class DataState(
        val encoder: Encoder,
        val limitSwitch: DigitalChannel,
        var ticks: Int = 0,
        var velocity: Double = 0.0,
        var acceleration: Double = 0.0
    ) : BaseCommandState() {
        val isResting: Boolean
            get() = limitSwitch.state
    }

    data class TargetingState(
        val motorL: DcMotorEx,
        val motorR: DcMotorEx,
        @Editable
        val manualPid: PID = PID(PIDTerms(16.0, 30.0, 0.0), 0.0, MAX_TICKS.toDouble(), -1.0, 1.0),
        var targetTicks: Int = 0
    ) : BaseCommandState()
}