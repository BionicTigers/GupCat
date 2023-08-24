package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

class IntakeOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val intake = Intake(hardwareMap)

        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            intake.start()
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onEnd {
            intake.stop()
        }

    }
}