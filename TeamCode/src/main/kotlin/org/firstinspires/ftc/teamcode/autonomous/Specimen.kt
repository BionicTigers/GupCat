package org.firstinspires.ftc.teamcode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.groups.concurrent
import io.github.bionictigers.axiom.commands.groups.sequential
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.time.Duration.Companion.seconds
import com.pedropathing.localization.Pose as PedroPose


class Specimen : LinearOpMode()  {
    private val startPose = Pose(8, 72, 90).toPedro()
    private val submersiblePose1 = Pose(26.78, 72, 180).toPedro()
    private val submersiblePose2 = Pose(26.78, 70, 180).toPedro()
    private val pickupPose = Pose(17.8, 40, 180).toPedro()
    private val parkPose = Pose(11.8, 25.89, 0).toPedro()

    private fun createScoringPath(follower: Follower, startPose: PedroPose, endPose: PedroPose): PathChain =
        follower.pathBuilder()
            .addPath(BezierLine(startPose, endPose))
            .setConstantHeadingInterpolation(endPose.heading)
            .build()

    private fun createParkingPath(follower: Follower): PathChain =
        follower.pathBuilder()
            .addPath(BezierLine(submersiblePose2, parkPose))
            .setTangentHeadingInterpolation()
            .build()

    override fun runOpMode() {
        Scheduler.telemetry = telemetry

        val arm = Arm(hardwareMap, telemetry)
        val claw = Claw(hardwareMap, telemetry)
        val pivot = Pivot(hardwareMap, telemetry)
        val slides = Slides(hardwareMap, pivot, telemetry)
        val drivetrain = Drivetrain(hardwareMap, telemetry, startPose)

        val follower = drivetrain.follower

        //Force initialize position
        Scheduler.schedule(claw.close(), arm.up())
        Scheduler.update()

        Scheduler.addSystem(arm, claw, pivot, slides, drivetrain)

        val score = sequential("Score") {
            add(pivot.max())
            waitUntil { pivot.angle > Angle.degrees(85) }
            add(slides.moveTo(30000))
            waitUntil { slides.ticks > 29500 }

            add(arm.down())
            concurrent {
                add(claw.open())
                add(pivot.min())
                add(slides.min())
            }
        }

        val specimenAuto = sequential("Specimen Autonomous") {
            sequential("Score Preload") {
                instant { follower.followPath(createScoringPath(follower, startPose, submersiblePose1)) }
                waitUntil { follower.currentTValue > 0.95 }
                add(score)
            }

            sequential("Pickup Second Specimen") {
                instant { follower.followPath(createScoringPath(follower, submersiblePose1, pickupPose)) }
                waitUntil { follower.currentTValue > 0.95 }
                add(claw.close())
                wait(.05.seconds)
                add(arm.up())
            }

            sequential("Score Second Specimen") {
                instant { follower.followPath(createScoringPath(follower, pickupPose, submersiblePose2)) }
                waitUntil { follower.currentTValue > 0.95 }
                add(score)
            }

            sequential("Park") {
                instant { follower.followPath(createParkingPath(follower)) }
                waitUntil { follower.currentTValue > 0.95 }
            }
        }

        waitForStart()

        Scheduler.schedule(specimenAuto)

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }
}