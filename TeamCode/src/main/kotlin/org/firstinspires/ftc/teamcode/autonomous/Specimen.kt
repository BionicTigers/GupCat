package org.firstinspires.ftc.teamcode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
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
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.time.Duration.Companion.seconds
import com.pedropathing.localization.Pose as PedroPose

@Autonomous
class Specimen : LinearOpMode()  {
    private val startPose = Pose(8, 72, 90).toPedro()
    private val submersiblePose1 = Pose(26.1, 72, 180).toPedro()
    private val submersiblePose2 = Pose(27.4, 70, 180).toPedro()
    private val submersiblePose3 = Pose(27.4, 76, 180).toPedro()
    private val humanPlayerPickup = Pose(17.6, 40, 180).toPedro()
    private val groundPickupLeft = Pose(26, 25.5, 15).toPedro()
    private val groundPickupMiddle = Pose(26, 25.5, 350).toPedro()
    private val groundPickupRight = Pose(26, 25.5, 325).toPedro()
    private val humanPlayerDropOff = Pose(25, 25.5, 180).toPedro()
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
        val slides = Slides(hardwareMap, pivot, telemetry, arm)
        val drivetrain = Drivetrain(hardwareMap, telemetry, startPose)

        val follower = drivetrain.follower

        //Force initialize position
        Scheduler.schedule(claw.close(), arm.up())
        Scheduler.update()

        Scheduler.addSystem(arm, claw, pivot, slides, drivetrain)

        val score = {
            sequential("Score") {
                add(pivot.max())
                waitUntil { pivot.angle > Angle.degrees(89) }
                add(slides.moveTo(30000))
                waitUntil { slides.ticks > 29500 }

                add(arm.down())
                concurrent {
                    add(claw.open())
                    add(pivot.min())
                    add(slides.min())
                }
            }
        }

        fun pickupFromHumanPlayer(startPose: PedroPose) =
            sequential("Pickup Second Specimen") {
                instant { follower.followPath(createScoringPath(follower, startPose, humanPlayerPickup)) }
                add(arm.specimen())
                waitUntil { follower.currentTValue > 0.98 }
                add(claw.close())
                wait(.05.seconds)
                add(arm.up())
                wait(.05.seconds)
            }

        val specimenAuto = sequential("Specimen Autonomous") {
            sequential("Score Preload") {
                instant { follower.followPath(createScoringPath(follower, startPose, submersiblePose1)) }
                waitUntil { follower.currentTValue > 0.92 }
                add(score())
            }

            add(pickupFromHumanPlayer(submersiblePose1))

            sequential("Score Second Specimen") {
                instant { follower.followPath(createScoringPath(follower, humanPlayerPickup, submersiblePose2)) }
                waitUntil { !follower.isBusy }
                add(score())
            }

            sequential("Pickup Left") {
                instant { follower.followPath(createScoringPath(follower, submersiblePose2, groundPickupLeft)) }
                waitUntil { follower.currentTValue > .97 }
                add(slides.moveTo(12000))
                waitUntil { slides.ticks > 11700 }
                add(claw.close())
            }

            sequential("Dropoff Left") {
                instant { follower.followPath(createScoringPath(follower, groundPickupLeft, humanPlayerDropOff)) }
                waitUntil { follower.currentTValue > .97 }
                add(claw.open())
                wait(.1.seconds)
                add(slides.min())
                wait(2.seconds)
            }

            add(pickupFromHumanPlayer(humanPlayerDropOff))

            sequential("Score Second Specimen") {
                instant { follower.followPath(createScoringPath(follower, humanPlayerPickup, submersiblePose3)) }
                waitUntil { !follower.isBusy }
                add(score())
            }

            add(arm.middle())

//            sequential("Pickup Middle") {
//                instant { follower.followPath(createScoringPath(follower, humanPlayerDropOff, groundPickupMiddle)) }
//                waitUntil { follower.currentTValue > .97 }
//                add(slides.moveTo(11000))
//                waitUntil { slides.ticks > 10700 }
//                add(claw.close())
//            }
//
//            sequential("Dropoff Middle") {
//                instant { follower.followPath(createScoringPath(follower, groundPickupMiddle, humanPlayerDropOff)) }
//                waitUntil { follower.currentTValue > .97 }
//                add(claw.open())
//            }
//
//            sequential("Pickup Right") {
//                instant { follower.followPath(createScoringPath(follower, humanPlayerDropOff, groundPickupRight)) }
//                waitUntil { follower.currentTValue > .97 }
//                add(slides.moveTo(12000))
//                waitUntil { slides.ticks > 11700 }
//                add(claw.close())
//            }
//
//            sequential("Dropoff Right") {
//                instant { follower.followPath(createScoringPath(follower, groundPickupRight, humanPlayerDropOff)) }
//                waitUntil { follower.currentTValue > .97 }
//                add(claw.open())
//            }

//            sequential("Park") {
//                instant { follower.followPath(createParkingPath(follower)) }
//                waitUntil { follower.currentTValue > 0.95 }
//            }
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