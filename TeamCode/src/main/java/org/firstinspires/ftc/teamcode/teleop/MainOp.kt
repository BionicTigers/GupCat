package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import kotlin.math.withSign

@TeleOp(name="MainOp")
class MainOp : LinearOpMode{
    override fun runOpMode() {

        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val drivetrain = Drivetrain(hardwareMap, robot)
        val lift = Lift(hardwareMap, robot)
        val arm = Arm(hardwareMap, robot)
        val claw = Claw(hardwareMap, robot)

        //drivetrain stuff
        gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK).onChange { pos ->
            gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).state?.let {
                drivetrain.robotDMP(pos, it.x)
            }
        }

        gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).onChange { pos ->
            gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK).state?.let {
                drivetrain.robotDMP(it, pos.x)
            }
        }

        //lift
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart{
            lift.targetHeight = 800
            lift.killPower = false
            println("triggrtrf")
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart{
            lift.targetHeight = 400
            lift.killPower = false
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart{
            lift.targetHeight = 0
            lift.killPower = false
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart{
            lift.targetHeight = 50
            lift.killPower = true
        }

        val rightJoystick = gamepad2.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        rightJoystick.deadzone = 0.3
        rightJoystick.onChange {
            lift.trim += it.y.withSign(300 * Scheduler.deltaTime)
        }

        //arm
        var power: Double = 0.0

        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            arm.move(150.0 * Scheduler.deltaTime)
        }

        //claw
        gamepad2.getButton(GamepadEx.Buttons.A).onStart{
            claw.open()
        }
        gamepad2.getButton(GamepadEx.Buttons.A).onEnd{
            claw.close()
        }
        if (claw.coneDetected()){
            println("cone")
        }

        //scheduler stuff
        Scheduler.add(ContinuousCommand { lift.update() })
        Scheduler.add(ContinuousCommand { arm.update() })


        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }
}