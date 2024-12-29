package org.firstinspires.ftc.teamcode.teleops.mechanisms

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Slides

@TeleOp(name = "SlideOp", group = "mechanisms")
class SlideOp : LinearOpMode() {
    override fun runOpMode() {
        val slides = Slides(hardwareMap)
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val (gp1, gp2) = gamepadSystem.gamepads
        slides.setupDriverControl(gp1)

        Scheduler.addSystem(slides, gamepadSystem)
        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
        }

        Scheduler.clear()
    }

}