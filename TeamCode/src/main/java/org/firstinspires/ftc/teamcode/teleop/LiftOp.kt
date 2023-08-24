package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name="LiftOp")
class LiftOp : LinearOpMode() {
    override fun runOpMode() {

        val robot = Robot(this)
        val (_, gamepad2) = robot.getGamepads()
        val lift = Lift(hardwareMap, robot)

        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart{
            lift.targetHeight = 800
            lift.killPower = false
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

        Scheduler.add(ContinuousCommand { lift.update() })

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }

        Scheduler.clear()
    }
}