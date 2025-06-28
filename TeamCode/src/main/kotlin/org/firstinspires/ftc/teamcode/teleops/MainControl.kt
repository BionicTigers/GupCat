package org.firstinspires.ftc.teamcode.teleops

import com.pedropathing.localization.PoseUpdater
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.profiles.Alex
import org.firstinspires.ftc.teamcode.input.profiles.Erin
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.CustomPedroLocalizer
import org.firstinspires.ftc.teamcode.pedro.FConstants
import org.firstinspires.ftc.teamcode.pedro.LConstants

@TeleOp
class MainControl : LinearOpMode() {
    private val profileGP1 = Alex
    private val profileGP2 = Erin

    override fun runOpMode() {
        Scheduler.telemetry = telemetry

        val arm = Arm(hardwareMap, telemetry)
        val claw = Claw(hardwareMap, telemetry)
        val pivot = Pivot(hardwareMap, telemetry)
        val slides = Slides(hardwareMap, pivot, telemetry)
        val drivetrain = Drivetrain(hardwareMap, telemetry)

        val controls = Controls(gamepad1, gamepad2, profileGP1, profileGP2, listOf(arm, claw, pivot, slides, drivetrain))

        Scheduler.addSystem(arm, claw, pivot, slides, controls, drivetrain)

        Scheduler.schedule(claw.open(), arm.down())

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }
}