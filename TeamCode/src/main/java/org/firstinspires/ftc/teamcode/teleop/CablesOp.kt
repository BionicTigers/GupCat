package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Cables
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name= "CablesOp")
class CablesOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, _) = robot.getGamepads()
        val cables = Cables(hardwareMap)

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            cables.lift()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onEnd {
            cables.stop()
        }

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }
}