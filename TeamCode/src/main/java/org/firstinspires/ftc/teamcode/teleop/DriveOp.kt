package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name="DriveOp")
class DriveOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val drivetrain = Drivetrain(hardwareMap, robot)

        val leftJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        val rightJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        leftJoystick.onChange { pos ->
            rightJoystick.state?.let {
                drivetrain.robotDMP(pos, -it.x)
            }
        }

        rightJoystick.onChange { pos ->
            leftJoystick.state?.let {
                drivetrain.robotDMP(it, -pos.x)
            }
        }

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }
}