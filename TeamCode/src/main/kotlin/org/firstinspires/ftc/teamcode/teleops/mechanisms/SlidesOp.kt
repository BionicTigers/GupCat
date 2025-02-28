package org.firstinspires.ftc.teamcode.teleops.mechanisms

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides

@TeleOp(name = "SlidesOp")
class SlidesOp : LinearOpMode() {
    override fun runOpMode() {
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val pivot = Pivot(hardwareMap)
        val slides = Slides(hardwareMap, pivot)
        val (gp1, gp2) = gamepadSystem.gamepads

        slides.setupDriverControl(gp1)
        pivot.setupDriverControl(gp1)

        Scheduler.addSystem(pivot, slides)

        waitForStart()

        while (true) {
            Scheduler.update()
        }
    }
}