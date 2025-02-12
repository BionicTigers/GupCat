package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Pivot

@TeleOp(name = "Test")
class Test : LinearOpMode() {
    override fun runOpMode() {
        val pivot = Pivot(hardwareMap)
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val (gp1, gp2) = gamepadSystem.gamepads

        Scheduler.addSystem(pivot)

        pivot.setupDriverControl(gp1)

        waitForStart()
        while (opModeIsActive()) {
            Scheduler.update()
        }
    }
}