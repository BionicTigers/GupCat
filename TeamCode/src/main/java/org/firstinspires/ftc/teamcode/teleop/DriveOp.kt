package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name="DriveOpMode")
class DriveOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val drivetrain = Drivetrain(hardwareMap, robot)

        gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK).onChange { pos ->
            gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).state?.let {
                drivetrain.robotDMP(pos, it.x) }
        }

        gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).onChange { pos ->
            gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK).state?.let {
                drivetrain.robotDMP(it, pos.x) }
        }

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }
}