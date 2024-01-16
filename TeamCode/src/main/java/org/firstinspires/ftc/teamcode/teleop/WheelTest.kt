package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name = "WheelTest", group = "Testing")
class WheelTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val drivetrain = Drivetrain(hardwareMap, robot)

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            drivetrain.driveFrontLeft() //FrontRight
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart {
            drivetrain.driveFrontRight() //BackRight
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            drivetrain.driveBackRight() //BackLeft
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart {
            drivetrain.driveBackLeft() //FrontLeft
        }

        gamepad1.getButton(GamepadEx.Buttons.A).onStart {
            drivetrain.stop()
        }

        robot.onStart { robot.update() }
    }
}