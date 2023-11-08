package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import org.firstinspires.ftc.teamcode.utils.vision.AprilTags

@TeleOp(name = "AprilTagOp")
class AprilTagOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (_, gamepad2) = robot.getGamepads()
        val aprilTags = AprilTags(robot, hardwareMap)

        gamepad2.getButton(GamepadEx.Buttons.A).onStart{
            aprilTags.aprilTagLog(aprilTags.calculateRobotPos(), telemetry)
        }

        gamepad2.getButton(GamepadEx.Buttons.B).onStart{
            telemetry.addLine("Apriltags in view: ${aprilTags.getAprilTagDetections()}")
        }

        gamepad2.getButton(GamepadEx.Buttons.Y).onStart{
            aprilTags.toggleLiveView()
        }

        gamepad2.getButton(GamepadEx.Buttons.X).onStart{
            aprilTags.toggleATProcessor()
        }


        waitForStart()

        while (opModeIsActive()) {
            aprilTags.calculateRobotPos()
            telemetry.update()
        }
    }
}