package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.getByName

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