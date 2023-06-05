package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import org.firstinspires.ftc.teamcode.mechanisms.lift
import org.firstinspires.ftc.teamcode.utils.Vector2

class LiftOp : LinearOpMode() {
    override fun runOpMode() {

        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val lift = lift(hardwareMap, robot)
        var vector = gamepad2.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).state

        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart{
            lift.nae(-800, false)
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_LEFT).onStart{
            lift.nae(-400, false)
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart{
            lift.nae(0, false)
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_RIGHT).onStart{
            lift.nae(50, true)
        }

        gamepad2.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).onChange{
            vector
            if (vector != null) {
                if(vector.magnitude() >= 0.3){
                    lift.notTrim(10)
                } else if(vector.magnitude() <= -0.3){
                    lift.notTrim(-10)
                }

            }
        }
    }
}