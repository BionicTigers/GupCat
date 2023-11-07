package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp()
class IntakeOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (_, gamepad2) = robot.getGamepads()
        val intake = Intake(hardwareMap)

        gamepad2.getButton(GamepadEx.Buttons.A).onStart {
            intake.start()
        }

        gamepad2.getButton(GamepadEx.Buttons.B).onStart {
            intake.stop()
        }

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }
}