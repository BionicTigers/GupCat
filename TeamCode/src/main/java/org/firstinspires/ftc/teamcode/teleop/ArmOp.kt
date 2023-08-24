package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name="ArmOp")
class ArmOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val arm = Arm(hardwareMap)

        gamepad2.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK).onChange {
            arm.power = -it.y
        }

        Scheduler.add(ContinuousCommand { arm.update() })

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }

        Scheduler.clear()
    }
}