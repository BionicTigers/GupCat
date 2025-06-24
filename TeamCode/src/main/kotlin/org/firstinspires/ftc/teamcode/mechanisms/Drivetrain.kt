package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.follower.Follower
import com.pedropathing.localization.PoseUpdater
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.input.ControlSchema
import org.firstinspires.ftc.teamcode.input.Controllable
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.Gamepads
import org.firstinspires.ftc.teamcode.input.Profile
import org.firstinspires.ftc.teamcode.input.matches
import org.firstinspires.ftc.teamcode.input.types.Analog
import org.firstinspires.ftc.teamcode.input.types.Control
import org.firstinspires.ftc.teamcode.motion.CustomPedroLocalizer
import org.firstinspires.ftc.teamcode.pedro.FConstants
import org.firstinspires.ftc.teamcode.pedro.LConstants

class Drivetrain(hardwareMap: HardwareMap, telemetry: Telemetry? = null) : System, Controllable {
    enum class DriveOrientation {
        /** Movement is relative to the robot */
        ROBOT,
        /** Movement is relative to the field, direction is constant */
        FIELD
    }

    interface Schema : ControlSchema {
        /** The method used to control the robot */
        val orientation: DriveOrientation

        /** Lateral Movement (Left-Right) */
        val x: Analog

        /** Longitudinal Movement (Forward-Backward) */
        val y: Analog

        /** Rotational Movement */
        val rot: Analog
    }

    companion object {
        init {
            Constants.setConstants(FConstants::class.java, LConstants::class.java)
        }
    }

    override val name = "drivetrain"

    val localizer = CustomPedroLocalizer(hardwareMap, telemetry = telemetry)
    val follower = Follower(hardwareMap, localizer, FConstants::class.java, LConstants::class.java)

    val data = DrivetrainData()

    fun setXControl(x: Double): Command<DrivetrainData> = Command.instant("Set X Control", data) {
        it.xControl = x
    }

    fun setYControl(y: Double): Command<DrivetrainData> = Command.instant("Set Y Control", data) {
        it.yControl = y
    }

    fun setRotControl(rot: Double): Command<DrivetrainData> = Command.instant("Set Rot Control", data) {
        it.rotControl = rot
    }

    init {
        if (telemetry != null) {
            Scheduler.schedule(Command.continuous("Drivetrain Log") {
                telemetry.addData("Drivetrain X", data.xControl)
                telemetry.addData("Drivetrain Y", data.yControl)
                telemetry.addData("Drivetrain Rot", data.rotControl)
            })
        }
    }

    override val afterRun = Command.continuous("Drivetrain Update", data) {
        if (it.driveOrientation != null) {
            follower.setTeleOpMovementVectors(it.yControl, it.xControl, it.rotControl, it.driveOrientation == DriveOrientation.ROBOT)
        }

        follower.update()
    }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.drivetrain) {
            if (!gamepad.matches(desiredGamepad)) return@with
            follower.startTeleopDrive()
            data.driveOrientation = orientation

            builder.register(x) { setXControl(it * x.modifier) }
            builder.register(y) { setYControl(it * y.modifier) }
            builder.register(rot) { setRotControl(it * rot.modifier) }
        }

    data class DrivetrainData(
        /* We only need to set these if we are using driver control */
        var driveOrientation: DriveOrientation? = null,
        var xControl: Double = 0.0,
        var yControl: Double = 0.0,
        var rotControl: Double = 0.0,
    ) : BaseCommandState()
}