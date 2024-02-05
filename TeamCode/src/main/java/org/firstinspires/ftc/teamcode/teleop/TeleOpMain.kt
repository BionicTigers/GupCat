package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Hang
import org.firstinspires.ftc.teamcode.mechanisms.Chainbar
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.Command
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

/**
 * Main TeleOp for the 2023-24 season, contains all mechanisms and matches functions to buttons
 */
@TeleOp
class TeleOpMain : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
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
        gamepad1.getTrigger(GamepadEx.Triggers.LEFT_TRIGGER).onHold {
            intake.start()
        }
        gamepad1.getTrigger(GamepadEx.Triggers.LEFT_TRIGGER).onEnd {
            intake.stop()
        }

        //When the down button on GP2 is pressed, the intake stops
        gamepad1.getTrigger(GamepadEx.Triggers.RIGHT_TRIGGER).onHold {
            intake.reverse()
        }
        gamepad1.getTrigger(GamepadEx.Triggers.RIGHT_TRIGGER).onEnd {
            intake.stop()
        }

        gamepad1.getButton(GamepadEx.Buttons.BACK).onStart {
            drivetrain.setJunkPosition()
        }

        //When the left trigger on GP2 is pressed, the intake is raised
        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            intake.down()
        }

        //When the right trigger on GP2 is pressed, the intake is lowered
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            intake.up()
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
        Scheduler.add(continuousCommand {
            println(leftJoystick.state)
            drivetrain.fieldDMP(leftJoystick.state!!, -rightJoystick.state!!.x)
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
//        gamepad2.getButton(GamepadEx.Buttons.Y).onHold {
//            slide.height += 250 * Scheduler.deltaTime
////            arm.up()
//        }
//
//        //When the X button is pressed on GP2, the target height of the slides is lowered by 500
//        gamepad2.getButton(GamepadEx.Buttons.X).onHold {
//            arm.down()
//            slide.height -= 250 * Scheduler.deltaTime
//        }
        //Button to decrement slide height to given position when pressed
        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart{
            slide.height -= 100
        }

        //Button to increment slide height to given position when pressed
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart{
            slide.height += 100
        }

        //Button to reset slide height to zero position when pressed
        gamepad2.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart{
            slide.height = 0.0
        }


        val leftGP2Joystick = gamepad2.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        Scheduler.add(continuousCommand {
            println(leftGP2Joystick.state)
            slide.height -= 1000 * Scheduler.deltaTime.seconds() * leftGP2Joystick.state!!.y
            println(leftGP2Joystick.state?.y)
            println(Scheduler.deltaTime)
        })

        Scheduler.add(continuousCommand { slide.update() })

        //output
        //When the up button on GP1 is pressed, the claw opens
        gamepad2.getButton(GamepadEx.Buttons.A).onStart {
            output.open()
        }


        //When the down button on GP                                                                                                                          1 is pressed, the claw closes
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

        //When the triangle button on GP1 is pressed, the drivetrain enters slow mode.
        gamepad1.getButton(GamepadEx.Buttons.Y).onStart {
            drivetrain.slow()
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

        Scheduler.add(Command {
            arm.down()
            chainbar.down()
        })

        robot.onStart{
            robot.update() //Updates position telemetry and gamepads
//            slide.update() //Runs slides to current target position
        }
}}