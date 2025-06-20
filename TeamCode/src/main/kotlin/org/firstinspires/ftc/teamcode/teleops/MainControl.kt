package org.firstinspires.ftc.teamcode.teleops

import com.pedropathing.localization.PoseUpdater
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.profiles.Alex
import org.firstinspires.ftc.teamcode.input.profiles.Erin
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.pedro.FConstants
import org.firstinspires.ftc.teamcode.pedro.LConstants

class MainControl : LinearOpMode() {
    val profileGP1 = Alex
    val profileGP2 = Erin

    override fun runOpMode() {
        Scheduler.telemetry = telemetry

        val arm = Arm(hardwareMap, telemetry)
        val claw = Claw(hardwareMap, telemetry)
        val pivot = Pivot(hardwareMap, telemetry)
        val slides = Slides(hardwareMap, pivot, telemetry)

        val controls = Controls(gamepad1, gamepad2, profileGP1, profileGP2, listOf(arm, claw, pivot, slides))

//        val poseUpdater = PoseUpdater(hardwareMap, localizer, FConstants::class.java, LConstants::class.java)

        Scheduler.addSystem(arm, claw, pivot, slides, controls)

        Scheduler.schedule(claw.open(), arm.down())

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }
}