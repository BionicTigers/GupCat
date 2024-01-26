package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name="SlideOp", group = "mechanisms")

class SlideOp : LinearOpMode() {
    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val robot = Robot(this)
        val (gamepad1, _) = robot.getGamepads()
        val slide = Slide(hardwareMap)
        val telemetry: Telemetry = robot.telemetry

        gamepad1.getButton(GamepadEx.Buttons.Y).onStart {
            slide.height += 500 * Scheduler.deltaTime.seconds()
        }

        gamepad1.getButton(GamepadEx.Buttons.X).onStart {
            slide.height -= 500 * Scheduler.deltaTime.seconds()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            slide.height = 1500.0
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            slide.height = 0.0
        }

        Scheduler.add(continuousCommand { slide.update() })

        waitForStart()

        while (opModeIsActive()) {
            robot.update()

        }

        Scheduler.clear()
    }
}