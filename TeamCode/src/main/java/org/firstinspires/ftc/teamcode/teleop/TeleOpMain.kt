package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Cables
import org.firstinspires.ftc.teamcode.mechanisms.Chainbar
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
//        val cables = Cables(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, robot)
//        val drone = Drone(hardwareMap)
        val intake = Intake(hardwareMap)
        val output = Output(hardwareMap)
        val slide = Slide(hardwareMap)
        val chainbar = Chainbar(hardwareMap)
        val arm = Arm(hardwareMap)

        val leftJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        val rightJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        //intake
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            intake.start()
        }

        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            intake.stop()
        }

        gamepad2.getTrigger(GamepadEx.Triggers.LEFT_TRIGGER).onStart {
            intake.up()
        }

        gamepad2.getTrigger(GamepadEx.Triggers.RIGHT_TRIGGER).onStart {
            intake.down()
        }

        //hanging
//        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
//            cables.lift()
//        }
//
//        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onEnd {
//            cables.stop()
//        }

        //drivetrain
        Scheduler.add(ContinuousCommand {
            drivetrain.robotDMP(leftJoystick.state!!, -rightJoystick.state!!.x)
        })


        //drone
//        gamepad2.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart {
//            drone.start()
//        }
//
//        gamepad2.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart {
//            drone.stop()
//        }

        //slide
        gamepad2.getButton(GamepadEx.Buttons.Y).onStart {
            slide.height += 500 * Scheduler.deltaTime
        }

        gamepad2.getButton(GamepadEx.Buttons.X).onStart {
            slide.height -= 500 * Scheduler.deltaTime
        }

        //output
        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            output.open()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            output.close()
        }

        //chainbar
        gamepad2.getButton(GamepadEx.Buttons.A).onStart {
            chainbar.up()
        }

        gamepad2.getButton(GamepadEx.Buttons.B).onStart {
            chainbar.down()
        }

        //arm
        gamepad2.getButton(GamepadEx.Buttons.LEFT_BUMPER).onStart {
            arm.up()
        }

        gamepad2.getButton(GamepadEx.Buttons.RIGHT_BUMPER).onStart {
            arm.down()
        }

        robot.onStart{
            robot.update()
            slide.update()
        }

        Scheduler.clear()


}}