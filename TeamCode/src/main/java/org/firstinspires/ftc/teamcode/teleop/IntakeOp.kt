package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp()
class IntakeOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (_, gamepad2) = robot.getGamepads()
        val intake = Intake(hardwareMap)
        val telemetry: Telemetry = robot.telemetry

        gamepad2.getButton(GamepadEx.Buttons.A).onStart {
            intake.start()
        }

        gamepad2.getButton(GamepadEx.Buttons.B).onStart {
            intake.stop()
        }

        gamepad2.getTrigger(GamepadEx.Triggers.LEFT_TRIGGER).onStart { intake.up() }
        gamepad2.getTrigger(GamepadEx.Triggers.RIGHT_TRIGGER).onStart { intake.down() }

        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart { intake.trimLeftDown() }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart { intake.trimLeftUp() }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart { intake.trimRightDown() }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart { intake.trimRightUp() }

        telemetry.addData("Right Position: ", intake.rightServo.position)
        telemetry.addData("Left Position: ", intake.leftServo.position)

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
            telemetry.update()
        }
    }
}