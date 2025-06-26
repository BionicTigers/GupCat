package pedroPathing.tuners_tests.localization

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.localization.PoseUpdater
import com.pedropathing.util.Constants
import com.pedropathing.util.DashboardPoseTracker
import com.pedropathing.util.Drawing
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.pedro.FConstants
import org.firstinspires.ftc.teamcode.pedro.LConstants

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 * You can adjust the target angle on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@Autonomous(name = "Turn Localizer Tuner", group = ".Localization")
class TurnTuner : OpMode() {
    private lateinit var poseUpdater: PoseUpdater
    private lateinit var dashboardPoseTracker: DashboardPoseTracker

    private lateinit var telemetryA: Telemetry

    /**
     * This initializes the PoseUpdater as well as the FTC Dashboard telemetry.
     */
    override fun init() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        poseUpdater = PoseUpdater(hardwareMap, FConstants::class.java, LConstants::class.java)

        dashboardPoseTracker = DashboardPoseTracker(poseUpdater)

        telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        telemetryA.addLine("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.")
        telemetryA.update()

        Drawing.drawRobot(poseUpdater!!.pose, "#4CAF50")
        Drawing.sendPacket()
    }

    /**
     * This updates the robot's pose estimate, and updates the FTC Dashboard telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        poseUpdater!!.update()

        telemetryA!!.addData("total angle", poseUpdater!!.totalHeading)
        telemetryA!!.addLine("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.")
        telemetryA!!.addData(
            "multiplier",
            ANGLE / (poseUpdater!!.totalHeading / poseUpdater!!.localizer.turningMultiplier)
        )

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50")
        Drawing.drawRobot(poseUpdater!!.pose, "#4CAF50")
        Drawing.sendPacket()
    }

    companion object {
        var ANGLE: Double = 2 * Math.PI
    }
}