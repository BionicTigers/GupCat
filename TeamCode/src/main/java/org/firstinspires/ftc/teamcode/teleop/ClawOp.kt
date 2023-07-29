package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name="ClawOp")
class ClawOp : LinearOpMode() {
    override fun runOpMode() {

        val robot = Robot(this)
        val (_, gamepad2) = robot.getGamepads()
        val claw = Claw(hardwareMap, robot)

        gamepad2.getButton(GamepadEx.Buttons.A).onStart{
            claw.open()
        }
        gamepad2.getButton(GamepadEx.Buttons.A).onEnd{
            claw.close()
        }

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }
}