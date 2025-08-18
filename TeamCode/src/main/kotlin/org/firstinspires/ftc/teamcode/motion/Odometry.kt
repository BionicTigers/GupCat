package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.getByName
import org.jetbrains.kotlinx.multik.ndarray.data.D2Array

interface RobotConfig {
    /**
     * How far sideways from the center the forward (X) odometry pod is.
     * Left of center is positive, right is negative.
     */
    val forwardOffset: Distance

    /**
     * How far forwards from the center the strafe (Y) odometry pod is.
     * Forward of center is positive, backwards is negative.
     */
    val strafeOffset: Distance
}

object Configs {
    object Test: RobotConfig {
        override val forwardOffset: Distance = Distance.mm(0.0) // TODO: Set actual value
        override val strafeOffset: Distance = Distance.mm(0.0) // TODO: Set actual value
    }

    object Main: RobotConfig {
        override val forwardOffset: Distance = Distance.mm(0.0) // TODO: Set actual value
        override val strafeOffset: Distance = Distance.mm(0.0) // TODO: Set actual value
    }
}

class Odometry(
    hardwareMap: HardwareMap,
    telemetry: Telemetry,
    startPose: Pose = Pose(0.0, 0.0, 0.0),
    private val config: RobotConfig = Configs.Main,
    override val name: String = "Odometry"
): System {
    private val pinpoint = hardwareMap.getByName<GoBildaPinpointDriver>("pinpoint")

    var position = startPose
    var velocity = Pose(0.0, 0.0, 0.0)

    init {
        pinpoint.setOffsets(config.forwardOffset.mm, config.strafeOffset.mm, DistanceUnit.MM)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED)
//        pinpoint.recalibrateIMU()
        pinpoint.setPosition(startPose.toPose2D())
    }

    /**
     * Takes .25 seconds, robot must be stationary for the whole .25 seconds.
     * It is recommended that the imu is recalibrated before each use (at the start of an auto or teleop).
     */
    val recalibrate: Command<BaseCommandState> = Command.instant("Odometry Recalibrate") {
        pinpoint.recalibrateIMU()
    }

    override val beforeRun = Command.continuous("Odometry Update") {
        pinpoint.update()

        position = Pose(
            pinpoint.getPosX(DistanceUnit.MM),
            pinpoint.getPosY(DistanceUnit.MM),
            pinpoint.getHeading(AngleUnit.RADIANS)
        )

        velocity = Pose(
            pinpoint.getVelX(DistanceUnit.MM),
            pinpoint.getVelY(DistanceUnit.MM),
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        )
    }

    private var loopTimeTooLowWarning: Telemetry.Line? = null
    private var loopTimeTooHighWarning: Telemetry.Line? = null

    override val afterRun = Command.continuous("Odometry Check") {
        if (pinpoint.loopTime < 500)
            if (loopTimeTooLowWarning == null)
                loopTimeTooLowWarning = telemetry.addLine("Loop time is lower than 500ms, odometry may not be accurate/something may be wrong with pods according to the GoBilda documentation.")
        else
            loopTimeTooLowWarning?.let {
                telemetry.removeLine(loopTimeTooLowWarning)
                loopTimeTooLowWarning = null
            }

        if (pinpoint.loopTime > 1100)
            if (loopTimeTooHighWarning == null)
                loopTimeTooHighWarning = telemetry.addLine("Loop time is higher than 1100ms, odometry may not be accurate/something may be wrong with pods according to the GoBilda documentation.")
            else
                loopTimeTooHighWarning?.let {
                    telemetry.removeLine(it)
                    loopTimeTooHighWarning = null
                }
    }
}