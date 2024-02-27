package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name="MTP")
class MTP : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val dashboard = FtcDashboard.getInstance()
        val dashTelemetry = dashboard.telemetry

        robot.pose = Pose(0.0, 0.0, 0.0)
        dashTelemetry.addData("xPV", 0)
        dashTelemetry.addData("yPV", 0)
        dashTelemetry.addData("xSP", 0)
        dashTelemetry.addData("ySP", 0)
        dashTelemetry.addData("xActualAccel", 0)
        dashTelemetry.addData("yActualAccel", 0)
        dashTelemetry.addData("xCommandedAccel", 0)
        dashTelemetry.addData("yCommandedAccel", 0)
        dashTelemetry.addData("xActualVelocity", 0)
        dashTelemetry.addData("yActualVelocity", 0)
        dashTelemetry.addData("xCommandedVelocity", 0)
        dashTelemetry.addData("yCommandedVelocity", 0)
        dashTelemetry.update()

        val cmd = CommandGroup()
//            .add(drivetrain.moveToPosition(Pose(0.0, 0.0, 180.0)))
            .add(drivetrain.moveToPosition(Pose(0.0, 2000.0, 0.0)))
            .build()
        Scheduler.add(cmd)

        waitForStart()
        while (opModeIsActive()) {
            robot.update()
        }

        Scheduler.clear()
    }
}