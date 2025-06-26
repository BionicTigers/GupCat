package org.firstinspires.ftc.teamcode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.groups.concurrent
import io.github.bionictigers.axiom.commands.groups.sequential
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.CustomPedroLocalizer
import org.firstinspires.ftc.teamcode.pedro.FConstants
import org.firstinspires.ftc.teamcode.pedro.LConstants
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose
import com.pedropathing.localization.Pose as PedroPose
import org.firstinspires.ftc.teamcode.utils.toPose
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "Sample")
class Sample : LinearOpMode() {
    //Pedro Poses use inches
    private val startPose = Pose(9, 110.8, 270).toPedro()
    private val scorePose = Pose(17.8, 121.7, 315).toPedro()
    private val sample1Pose = Pose(30.5, 119, 0).toPedro() // Right
    private val sample2Pose = Pose(30.5, 124, 0).toPedro() // Middle
    private val sample3Pose = Pose(22, 124.5, 0).toPedro() // Left

    private fun createScoringPath(follower: Follower, startPose: PedroPose): PathChain =
        follower.pathBuilder()
            .addPath(BezierLine(startPose, scorePose))
            .setConstantHeadingInterpolation(scorePose.heading)
            .build()

    private fun createPickupPath(follower: Follower, endPose: PedroPose): PathChain =
        follower.pathBuilder()
            .addPath(BezierLine(scorePose, endPose))
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

        val score = { concurrent("Score") {
            add(arm.down())
            add(pivot.max())
            sequential {
                waitUntil { pivot.angle > Angle.degrees(25) }
                add(slides.max())
            }
            sequential {
                waitUntil { pivot.angle > Angle.degrees(85) && slides.ticks > Slides.MAX_TICKS - 3000 && follower.currentTValue > 0.9 }
                add(arm.up())
                wait(.575.seconds)
                add(claw.open())
                wait(.05.seconds)
            }
        } }

        val reset = {
            sequential("Reset") {
                add(arm.down())
                wait(.2.seconds)
                concurrent {
                    add(slides.min())
                    sequential {
                        waitUntil { slides.ticks < Slides.MAX_TICKS - 24000 }
                        add(pivot.min())
                    }
                }
            }
        }

        val commandGroup = sequential("Sample Autonomous") {
            sequential("Score Preload") {
                instant { follower.followPath(createScoringPath(follower, startPose)) }
                waitUntil { follower.currentTValue > 0.7 }
                add(score())
            }

            sequential("Pickup First Sample") {
                add(reset())
                instant { follower.followPath(createPickupPath(follower, sample1Pose)) }
                waitUntil { follower.currentTValue > 0.95 }
                add(claw.close())
                wait(.05.seconds)
            }

            sequential("Score First Sample") {
                instant { follower.followPath(createScoringPath(follower, sample1Pose)) }
                waitUntil { follower.currentTValue > 0.7 }
                add(score())
            }

            sequential("Pickup Second Sample") {
                add(reset())
                instant { follower.followPath(createPickupPath(follower, sample2Pose)) }
                waitUntil { follower.currentTValue > 0.95 }
                add(claw.close())
                wait(.05.seconds)
            }

            sequential("Score Second Sample") {
                instant { follower.followPath(createScoringPath(follower, sample2Pose)) }
                waitUntil { follower.currentTValue > 0.7 }
                add(score())
            }

            sequential("Pickup Third Sample") {
                add(reset())
                instant { follower.followPath(createPickupPath(follower, sample3Pose)) }
                add(slides.moveTo(20000))
                waitUntil { slides.ticks < Slides.PIVOT_RESTING_MAX_TICKS - 3000 && follower.currentTValue > 0.95 }
                add(claw.close())
                wait(.05.seconds)
                add(reset())
            }

            sequential("Score Third Sample") {
                instant { follower.followPath(createScoringPath(follower, sample3Pose)) }
                waitUntil { follower.currentTValue > 0.7 }
                add(score())
            }

            add(reset())
        }


        waitForStart()

        Scheduler.schedule(commandGroup)

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }
}