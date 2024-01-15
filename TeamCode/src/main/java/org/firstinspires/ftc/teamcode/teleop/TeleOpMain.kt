package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Hang
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

/**
 * Main TeleOp for the 2023-24 season, contains all mechanisms and matches functions to buttons
 */
@TeleOp
class TeleOpMain : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val cables = Hang(hardwareMap)
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
        //When the up button on GP2 is pressed, the intake starts
        gamepad1.getTrigger(GamepadEx.Triggers.LEFT_TRIGGER).onStart {
            intake.start()
        }

        //When the down button on GP2 is pressed, the intake stops
        gamepad1.getTrigger(GamepadEx.Triggers.RIGHT_TRIGGER).onStart {
            intake.stop()
        }

        //When the left trigger on GP2 is pressed, the intake is raised
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            intake.up()
        }

        //When the right trigger on GP2 is pressed, the intake is lowered
        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            intake.down()
        }

        //hanging
        //When the down button on GP1 is pressed, the hanging pulls down on the bar
        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            cables.pull()
        }

        //When the up button on GP1 is pressed, the hanging mechanism raises up
        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            cables.raise()
        }

        //When the A (X) button on GP1 is pressed, the hanging mechanism stops
        gamepad1.getButton(GamepadEx.Buttons.A).onStart {
            cables.stop()
        }

        //drivetrain
        //Uses current joystick positions to determine the correct motor powers
        Scheduler.add(ContinuousCommand {
            drivetrain.robotDMP(leftJoystick.state!!, -rightJoystick.state!!.x)
        })

        //drone
        //When the right button on GP2 is pressed, the drone flywheel runs
//        gamepad2.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart {
//            drone.start()
//        }
//        When the left button on GP2 is pressed, the flywheel stops
//        gamepad2.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart {
//            drone.stop()
//        }

        //slide
        //When the Y button is pressed on GP2, the target height of the slides is raised by 500
        gamepad2.getButton(GamepadEx.Buttons.Y).onHold {
            println(slide.height)
            println(Scheduler.deltaTime)
            slide.height += 250 * Scheduler.deltaTime
//            arm.up()
        }

        //When the X button is pressed on GP2, the target height of the slides is lowered by 500
        gamepad2.getButton(GamepadEx.Buttons.X).onHold {
            arm.down()
            slide.height -= 250 * Scheduler.deltaTime
        }

        Scheduler.add(ContinuousCommand({ slide.update() }))

        //output
        //When the up button on GP1 is pressed, the claw opens
        gamepad2.getButton(GamepadEx.Buttons.A).onStart {
            output.open()
        }

        //When the down button on GP1 is pressed, the claw closes
        gamepad2.getButton(GamepadEx.Buttons.B).onStart {
            output.close()
        }

        //chainbar
        //When the A button on GP2 is pressed, the chainbar raises
        gamepad2.getTrigger(GamepadEx.Triggers.LEFT_TRIGGER).onStart {
            chainbar.up()
        }

        //When the B button on GP2 is pressed, the chainbar lowers
        gamepad2.getTrigger(GamepadEx.Triggers.RIGHT_TRIGGER).onStart {
            chainbar.down()
        }

        //arm
        //When the left bumper on GP2 is pressed, the arm raises
        gamepad2.getButton(GamepadEx.Buttons.LEFT_BUMPER).onStart {
            arm.up()
        }

        //When the right bumper on GP2 is pressed, the arm lowers
        gamepad2.getButton(GamepadEx.Buttons.RIGHT_BUMPER).onStart {
            arm.down()
        }

        robot.onStart{
            robot.update() //Updates position telemetry and gamepads
//            slide.update() //Runs slides to current target position
        }

        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
}}