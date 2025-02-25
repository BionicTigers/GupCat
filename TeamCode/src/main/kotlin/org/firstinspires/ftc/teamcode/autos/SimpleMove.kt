package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.utils.Timer
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose
import io.github.bionictigers.axiom.utils.Time

@Autonomous(name = "SimpleMTP")
class SimpleMove : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)
//        val pivot = Pivot(hardwareMap)
//        val slides = Slides(hardwareMap)

        val dashboard = FtcDashboard.getInstance()
        val dashboardTelemetry = dashboard.telemetry

        Scheduler.addSystem(odometrySystem, drivetrain)


        val poseToMove = Pose(0, 0, 180)
        val otherToMove = Pose(0, 0, 0)

        Persistents.reset()

        val otherMove = statelessCommand("move")
            .setOnEnter {
                drivetrain.moveToPose(otherToMove, Time.fromSeconds(4))
            }
            .setAction {
                dashboardTelemetry.addData("xPV", odometrySystem.globalPose.x)
                dashboardTelemetry.addData("yPV", odometrySystem.globalPose.y)
                dashboardTelemetry.addData("rPV", odometrySystem.globalPose.rotation.degrees)
                dashboardTelemetry.addData("xSP", 0)
                dashboardTelemetry.addData("ySP", 0)
                dashboardTelemetry.addData("rSP", poseToMove.rotation.degrees)
                dashboardTelemetry.update()
                drivetrain.moveFinished
            }

        val timer = Timer(Time.fromSeconds(3))
        Scheduler.add(
            statelessCommand("move")
                .setOnEnter {
                    drivetrain.moveToPose(poseToMove, Time.fromSeconds(4))
                }
                .setAction {
                    timer.update(it).finished {
                        println("Mrs Chast")
                    }
                    dashboardTelemetry.addData("xPV", odometrySystem.globalPose.x)
                    dashboardTelemetry.addData("yPV", odometrySystem.globalPose.y)
                    dashboardTelemetry.addData("rPV", odometrySystem.globalPose.rotation.degrees)
//                    dashboardTelemetry.addData("xSP", drivetrain.beforeRun.state.profileX!!.getPosition(drivetrain.beforeRun.state.timeInScheduler - drivetrain.beforeRun.state.timeStarted))
//                    dashboardTelemetry.addData("ySP", drivetrain.beforeRun.state.profileY!!.getPosition(drivetrain.beforeRun.state.timeInScheduler - drivetrain.beforeRun.state.timeStarted))
                    dashboardTelemetry.addData("rSP", poseToMove.rotation.degrees)
                    dashboardTelemetry.update()
                    it.timeInScheduler > Time.fromSeconds(10)
                }
                .setOnExit {
//                    Scheduler.add(otherMove)
                }
        )

//        dashboardTelemetry.addData("xPV", odometrySystem.globalPose.x)
//        dashboardTelemetry.addData("yPV", odometrySystem.globalPose.y)
//        dashboardTelemetry.addData("rPV", odometrySystem.globalPose.rotation.degrees)
//        dashboardTelemetry.addData("xSP", poseToMove.x)
//        dashboardTelemetry.addData("ySP", poseToMove.y)
//        dashboardTelemetry.addData("rSP", poseToMove.rotation.degrees)
//        dashboardTelemetry.update()

        dashboardTelemetry.addData("xVPV", 0)
        dashboardTelemetry.addData("yVPV", 0)
        dashboardTelemetry.addData("rVPV", 0)
        dashboardTelemetry.addData("xPSP", 0)
        dashboardTelemetry.addData("yPSP", 0)
        dashboardTelemetry.addData("rPSP", 0)
        dashboardTelemetry.addData("xPower", 0)
        dashboardTelemetry.addData("yPower", 0)
        dashboardTelemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            odometrySystem.log(telemetry)

            telemetry.update()
        }
    }
}