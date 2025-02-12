package org.firstinspires.ftc.teamcode.teleops.mechanisms

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Arm

@TeleOp(name="ArmOp", group = "mechanisms")
class ArmOp : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val arm = Arm(hardwareMap)

        Scheduler.addSystem(gamepadSystem)
        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
        }
        Scheduler.clear()
    }
}
