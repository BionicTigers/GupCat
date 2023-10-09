package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Cables
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Drone
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Lift
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
        val lift = Lift(hardwareMap, robot)
        val output = Output(hardwareMap)
        val slide = Slide(hardwareMap)

        val leftJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        val rightJoystick = gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        gamepad2.getButton(GamepadEx.Buttons.A).onStart {
            intake.start()
        }

        gamepad2.getButton(GamepadEx.Buttons.B).onStart {
            intake.stop()
        }

        gamepad2.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).onChange {
            arm.power = -it.y
        }

        Scheduler.add(ContinuousCommand { arm.update() })

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            cables.lift()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onEnd {
            cables.stop()
        }

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

        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            drone.start()
        }

        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            drone.stop()
        }

        gamepad2.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart{
            lift.targetHeight = 800
            lift.killPower = false
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart{
            lift.targetHeight = 400
            lift.killPower = false
        }
        gamepad2.getButton(GamepadEx.Buttons.X).onStart{
            lift.targetHeight = 0
            lift.killPower = false
        }
        gamepad2.getButton(GamepadEx.Buttons.Y).onStart{
            lift.targetHeight = 50
            lift.killPower = true
        }

        Scheduler.add(ContinuousCommand { lift.update() })

        gamepad1.getButton(GamepadEx.Buttons.Y).onStart {
            slide.height += 500 * Scheduler.deltaTime
        }

        gamepad1.getButton(GamepadEx.Buttons.X).onStart {
            slide.height -= 500 * Scheduler.deltaTime
        }
//left off
        gamepad2.getButton(GamepadEx.Buttons.T).onStart {
            output.openLeft()
        }

        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            output.openRight()
        }

        gamepad2.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart {
            output.close()
        }
        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }


}