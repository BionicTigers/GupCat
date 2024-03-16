package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Hang
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name= "CablesOp", group="disabled")
class CablesOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, _) = robot.getGamepads()
        val hang = Hang(hardwareMap)

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onEnd {
            hang.stop()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
        }

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }
}