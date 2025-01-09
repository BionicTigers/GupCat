package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.axiom.Timer
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.DrivetrainState
import org.firstinspires.ftc.teamcode.motion.Motors
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Time
import kotlin.math.abs

interface HighBasketScore : CommandState {
    val pivotTimer: Timer
    val dropTimer: Timer

    companion object {
        fun default(): HighBasketScore {
            return object : HighBasketScore, CommandState by CommandState.default("High Basket") {
                override val pivotTimer = Timer(Time.fromSeconds(.7))
                override val dropTimer = Timer(Time.fromSeconds(.7))
            }
        }
    }
}

interface PickupTime : CommandState {
    val hoverTimer: Timer

    companion object {
        fun default(): PickupTime {
            return object : PickupTime, CommandState by CommandState.default("Ground") {
                override val hoverTimer = Timer(Time.fromSeconds(.4))
            }
        }
    }
}

interface LeftPickupTime : PickupTime {
    val liftTime: Timer

    companion object {
        fun default(): LeftPickupTime {
            return object : LeftPickupTime, CommandState by PickupTime.default() {
                override val liftTime = Timer(Time.fromSeconds(1.2))
                override val hoverTimer = Timer(Time.fromSeconds(.4))
            }
        }
    }
}

@Autonomous
class BlueLeftPreload : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()

        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)
        val pivot = Pivot(hardwareMap)
        val slides = Slides(hardwareMap, pivot)
        val claw = Claw(hardwareMap)
        val arm = Arm(hardwareMap)

        Scheduler.addSystem(odometrySystem, drivetrain, pivot, slides)

        odometrySystem.globalPose = Pose(850.9, 215.9, 0)
        val highBasketPosition = Pose(410.8, 500.8, 45)
        val highBasketHeight = 3500
        val highBasketPivot = 1800

        val groundRightPosition = Pose(600, 860, 0)
        val groundMiddlePosition = Pose(359, 860, 0)

        val groundLeftPosition = Pose(450, 500, -31)
        val leftSlidesHeight = 2000

        val scoreHighBasket = Command(HighBasketScore.default())
        val retractFromBasket = statelessCommand("RetractFromBasket")
        val groundRight = Command(PickupTime.default())
        val groundMiddle = Command(PickupTime.default())
        val groundLeft = Command(LeftPickupTime.default())

        var samplesCollected = 0

        var moveClaw = false
        var driveMoveFinished = false

        Persistents.reset()

        scoreHighBasket
            .setOnEnter {
                claw.open = false
                arm.target = Arm.Position.Down
                drivetrain.moveToPose(highBasketPosition, Time.fromSeconds(5))
//                273, 453
            }
            .setAction {
                telemetry.addData("Move", "HighBasket")
                if (drivetrain.moveFinished || driveMoveFinished) {
                    driveMoveFinished = true
                    pivot.pivotTicks = highBasketPivot
                    if (highBasketPivot - pivot.pivotTicks <= 100 && it.pivotTimer.update(it).isFinished) {
                        slides.targetPosition = highBasketHeight
                        if (slides.targetPosition - slides.ticks <= 900) {
                            arm.target = Arm.Position.Up
                            moveClaw = true
                        }
                    }
                }

                if (moveClaw) {
                    it.dropTimer.update(it).finished {
                        println("claw open")
                        claw.open = true
                    }
                }

                driveMoveFinished && highBasketPivot - pivot.pivotTicks <= 100 && slides.targetPosition - slides.ticks <= 250 && it.dropTimer.isFinished
            }
            .setOnExit {
                println("Score End")
                Scheduler.add(retractFromBasket)
            }

        retractFromBasket
            .setOnEnter {
                arm.target = Arm.Position.Down
            }
            .setAction {
                telemetry.addData("Move", "Retract")
                slides.targetPosition = 0
                if (slides.ticks <= 400) {
                    pivot.pivotTicks = 0
                }

                pivot.pivotTicks <= 100 && abs(slides.targetPosition - slides.ticks) <= 250
            }
            .setOnExit {
                if (samplesCollected == 0) {
                    Scheduler.add(groundRight)
                } else if (samplesCollected == 1) {
                    Scheduler.add(groundMiddle)
                } else if (samplesCollected == 2) {
                    Scheduler.add(groundLeft)
                }

                samplesCollected++
            }

        groundRight
            .setOnEnter {
                println("Moving to Right")
                drivetrain.moveToPose(groundRightPosition, Time.fromSeconds(2.2))
            }
            .setAction {
                telemetry.addData("Move", "Ground Right")
                if (drivetrain.moveFinished) {
                    claw.open = false
                }

                drivetrain.moveFinished && it.hoverTimer.update(it).isFinished
            }
            .setOnExit {
                scoreHighBasket.state.dropTimer.reset()
                scoreHighBasket.state.pivotTimer.reset()
                driveMoveFinished = false
                moveClaw = false
                Scheduler.add(scoreHighBasket)
            }

        groundMiddle
            .setOnEnter {
                drivetrain.moveToPose(groundMiddlePosition, Time.fromSeconds(2.2))
            }
            .setAction {
                telemetry.addData("Move", "Ground Middle")
                if (drivetrain.moveFinished) {
                    claw.open = false
                }

                drivetrain.moveFinished && it.hoverTimer.update(it).isFinished
            }
            .setOnExit {
                scoreHighBasket.state.dropTimer.reset()
                scoreHighBasket.state.pivotTimer.reset()
                driveMoveFinished = false
                moveClaw = false
                Scheduler.add(scoreHighBasket)
            }

        var bringIn = false
        groundLeft
            .setOnEnter {
                drivetrain.moveToPose(groundLeftPosition, Time.fromSeconds(2.2))
            }
            .setAction {
                telemetry.addData("Move", "Ground Middle")
                if (it.liftTime.update(it).isFinished) {
                    slides.targetPosition = leftSlidesHeight
                    if (drivetrain.moveFinished && slides.targetPosition - slides.ticks <= 50) {
                        claw.open = false
                        bringIn = true
                    }
                }

                if (bringIn) {
                    slides.targetPosition = 0
                }

                println(leftSlidesHeight - slides.ticks)
                drivetrain.moveFinished && slides.targetPosition - slides.ticks <= 50 && it.hoverTimer.update(it).isFinished && it.liftTime.update(it).isFinished && bringIn
            }
            .setOnExit {
                scoreHighBasket.state.dropTimer.reset()
                scoreHighBasket.state.pivotTimer.reset()
                driveMoveFinished = false
                moveClaw = false
                Scheduler.add(scoreHighBasket)
            }

        Scheduler.add(scoreHighBasket)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            odometrySystem.log(telemetry)
            claw.logClaw(telemetry)
            arm.log(telemetry)
            slides.log(telemetry)
            pivot.log(telemetry)
        }
    }
}