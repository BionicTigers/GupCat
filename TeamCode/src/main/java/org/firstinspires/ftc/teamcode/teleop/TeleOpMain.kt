package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Cables
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Drone
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp
class TeleOpMain : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()

        val arm = Arm(hardwareMap)
        val cables = Cables(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val drone = Drone(hardwareMap)
        val intake = Intake(hardwareMap)
        val output = Output(hardwareMap)
        val slide = Slide(hardwareMap)

        val leftJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        val rightJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        //intake
        gamepad1.getButton(GamepadEx.Buttons.A).onStart {
            intake.start()
        }

        gamepad1.getButton(GamepadEx.Buttons.B).onStart {
            intake.stop()
        }

        //arm
        gamepad2.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).onChange {
            arm.power = -it.y
        }

        Scheduler.add(ContinuousCommand { arm.update() })

        //cables
        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            cables.lift()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onEnd {
            cables.stop()
        }

        //drivetrain
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


        //drone
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            drone.start()
        }

        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            drone.stop()
        }

        //slide
        gamepad1.getButton(GamepadEx.Buttons.Y).onStart {
            slide.height += 500 * Scheduler.deltaTime
        }

        gamepad1.getButton(GamepadEx.Buttons.X).onStart {
            slide.height -= 500 * Scheduler.deltaTime
        }

        //output
        gamepad1.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart {
            output.openLeft()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            output.openRight()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            output.close()
        }
        robot.onStart { }
    }


}