package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import io.github.bionictigers.axiom.utils.Time
import io.github.bionictigers.axiom.utils.Timer
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.math.abs

@Autonomous(name="Specimen")
class Specimen : LinearOpMode() {
    private val startingPose = Pose(1829, 203.1, 90)
    private val submersiblePose = Pose(1829, 680.1, 180)
    private val pickupPose = Pose(2643, 400, 180)
    private val parkPose = Pose(3000, 300, 0)

    private interface MoveToSubmersible : CommandState {
        val pivotTimer: Timer

        companion object {
            fun default(): MoveToSubmersible {
                return object : MoveToSubmersible, CommandState by CommandState.default("MoveToSubmersible") {
                    override val pivotTimer = Timer(Time.fromSeconds(1))
                }
            }
        }
    }

    private interface PickupFromWall : CommandState {
        val grabTimer: Timer
        val armTimer: Timer

        companion object {
            fun default(): PickupFromWall {
                return object : PickupFromWall, CommandState by CommandState.default("PickupFromWall") {
                    override val grabTimer = Timer(Time.fromSeconds(.4))
                    override val armTimer = Timer(Time.fromSeconds(.4))
                }
            }
        }
    }

    override fun runOpMode() {
        Persistents.reset()
        val slides = Slides(hardwareMap)
        val pivot = Pivot(hardwareMap, slides)
        slides.pivot = pivot

        val odometrySystem = OdometrySystem(hardwareMap, startingPose)
        val drivetrain = Drivetrain(hardwareMap, odometrySystem = odometrySystem, isAuto = true, sample = false)

        val claw = Claw(hardwareMap, 0.0)
        val arm = Arm(hardwareMap)

        val wallMove = statelessCommand("MoveFromWall")
        val moveToSubmersible = Command(MoveToSubmersible.default())
        val scoreElement = statelessCommand("ScoreSpecimen")
        val pickupFromWall = Command(PickupFromWall.default())
        val park = statelessCommand("Park")

//        wallMove
//            .setOnEnter { drivetrain.moveToPose(startingPose + Pose(0, 200, 0)) }
//            .setAction { drivetrain.moveFinished }
//            .setOnExit { Scheduler.add(moveToSubmersible) }

        var moves = 0
        moveToSubmersible
            .setOnEnter {
                claw.open = false
                arm.target = Arm.Position.Up
                drivetrain.moveToPose(submersiblePose + Pose(moves * 150, 15, 0))
                slides.mpMove(4000)
                it.pivotTimer.reset()
            }
            .setAction {
                if (drivetrain.moveFinished) {
                    pivot.mpSetPosition(pivot.max)
                }

                if (abs(pivot.ticks - pivot.max) < 300) it.pivotTimer.update(it)
                drivetrain.moveFinished && abs(pivot.ticks - pivot.max) < 300 && it.pivotTimer.isFinished
            }
            .setOnExit {
                moves++
                Scheduler.add(scoreElement)
            }

        scoreElement
            .setOnEnter {
                slides.beforeRun.state.profile = null
                slides.targetPosition = 30000
            }
            .setAction {
                if (abs(slides.ticks - slides.targetPosition) < 3000) {
                    arm.target = Arm.Position.Down
                }
                it.timeInScheduler > Time.fromSeconds(1.0)
            }
            .setOnExit {
                claw.open = true
                if (moves < 2) Scheduler.add(pickupFromWall) else Scheduler.add(park)
            }

        pickupFromWall
            .setOnEnter {
                slides.mpMove(0)
                pivot.mpSetPosition(0)
                arm.target = Arm.Position.Middle
                drivetrain.moveToPose(pickupPose)
                it.grabTimer.reset()
            }
            .setAction {
                if (drivetrain.moveFinished) {
                    slides.mpMove(6000)
                    if (abs(6000 - slides.ticks) < 500 || !claw.open) {
                        claw.open = false
                        if (it.grabTimer.update(it).isFinished) {
                            arm.target = Arm.Position.Up
                            it.armTimer.update(it)
                        }
                    }
                }
                drivetrain.moveFinished && !claw.open && abs(6000 - slides.ticks) < 500 && arm.target == Arm.Position.Up && it.armTimer.isFinished
            }
            .setOnExit {
                Scheduler.add(moveToSubmersible)
            }

        park
            .setOnEnter {
                drivetrain.moveToPose(parkPose)
                slides.mpMove(0)
                pivot.mpSetPosition(0)
            }
            .setAction { drivetrain.moveFinished }

        Scheduler.addSystem(slides, pivot, odometrySystem, drivetrain, claw, arm)
        Scheduler.add(moveToSubmersible)

        claw.open = false

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            odometrySystem.logPosition(telemetry)
            telemetry.update()
        }
    }
}