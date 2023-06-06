package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import kotlin.math.withSign

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

        rightJoystick.deadzone = 0.3
        rightJoystick.onChange {
            lift.trim += it.y.withSign(300 * Scheduler.deltaTime)
        }
    }
}