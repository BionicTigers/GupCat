package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.utils.Timer
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose
import io.github.bionictigers.axiom.utils.Time
import kotlin.math.abs

interface HighBasketScore : CommandState {
    val pivotTimer: Timer
    val dropTimer: Timer
    val moveTimer: Timer

    companion object {
        fun default(): HighBasketScore {
            return object : HighBasketScore, CommandState by CommandState.default("High Basket") {
                override val pivotTimer = Timer(Time.fromSeconds(.2))
                override val dropTimer = Timer(Time.fromSeconds(.9))
                override val moveTimer = Timer(Time.fromSeconds(3))
            }
        }
    }
}

interface Retract : CommandState {
    val slideEndTimer: Timer

    companion object {
        fun default(): Retract {
            return object : Retract, CommandState by CommandState.default("Retract") {
                override val slideEndTimer = Timer(Time.fromSeconds(.1))
            }
        }
    }
}

interface PickupTime : CommandState {
    val hoverTimer: Timer

    companion object {
        fun default(): PickupTime {
            return object : PickupTime, CommandState by CommandState.default("Ground") {
                override val hoverTimer = Timer(Time.fromSeconds(.075))
            }
        }
    }
}

interface LeftPickupTime : PickupTime {
    val liftTime: Timer
    val driveTimer: Timer

    companion object {
        fun default(): LeftPickupTime {
            return object : LeftPickupTime, CommandState by PickupTime.default() {
                override val driveTimer = Timer(Time.fromSeconds(.9))
                override val liftTime = Timer(Time.fromSeconds(.8))
                override val hoverTimer = Timer(Time.fromSeconds(.15))
            }
        }
    }
}

@Autonomous
class Sample : LinearOpMode() {
    override fun runOpMode() {
        Persistents.reset()
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap, Pose(812.9, 203.1, 90))
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem, isAuto = true, sample = true)
        val slides = Slides(hardwareMap)
        val pivot = Pivot(hardwareMap, slides)
        slides.pivot = pivot
        val claw = Claw(hardwareMap, 0.0)
        val arm = Arm(hardwareMap)

        Scheduler.addSystem(odometrySystem, drivetrain, pivot, slides)

        val highBasketPosition = Pose(502.8, 502.8, 45)
        val highBasketHeight = 52000
        val highBasketPivot = pivot.max

        val leftSlidesHeight = 28000

        val groundRightPosition = Pose(460.8, 505.8, 18) // 460
        val groundMiddlePosition = Pose(460, 505, -6)

        val groundLeftPosition = Pose(460, 505, -23.5)

        val moveForward = statelessCommand("moveForward")
        val scoreHighBasket = Command(HighBasketScore.default())
        val retractFromBasket = Command(Retract.default())
        val groundRight = Command(PickupTime.default())
        val groundMiddle = Command(PickupTime.default())
        val groundLeft = Command(LeftPickupTime.default())

        var samplesCollected = 0

        var moveClaw = false
        var driveMoveFinished = false

//        val tAfkf = Timer(Time.fromSeconds(3))
        moveForward
            .setOnEnter {
                claw.open = false
                drivetrain.moveToPose(Pose(700.9, 505.1, 90))
            }
            .setAction {
                return@setAction it.timeInScheduler >= Time.fromSeconds(.6)
            }
            .setOnExit {
                Scheduler.add(scoreHighBasket)
            }

        var pivotShouldMove = true
        var slidesShouldMove = true
        var moveStart = false

        scoreHighBasket
            .setOnEnter {
                claw.open = false
                arm.target = Arm.Position.Down
                slides.mpMove(16000)
                it.pivotTimer.reset()
                it.dropTimer.reset()
                it.moveTimer.reset()
                moveStart = true
//                273, 453
            }
            .setAction {
                if (moveStart) {
                    drivetrain.moveToPose(highBasketPosition, Time.fromSeconds(5))
                    moveStart = false
                }

                telemetry.addData("Move", "HighBasket")
                telemetry.addData("timer", it.moveTimer.duration)
                it.moveTimer.update(it)
                if (!(drivetrain.moveFinished || driveMoveFinished || it.moveTimer.isFinished)) {
                    pivot.mpSetPosition((highBasketPivot / 1.5).toInt())
                    slides.mpMove(22000)
                }
                if (drivetrain.moveFinished || driveMoveFinished || it.moveTimer.isFinished) {
                    if (!driveMoveFinished) pivot.mpSetPosition(highBasketPivot)
                    driveMoveFinished = true
                    if (highBasketPivot - pivot.pivotTicks <= 1600 /*&& it.pivotTimer.update(it).isFinished*/) {
                        slides.mpMove(highBasketHeight)
                        if (highBasketHeight - slides.ticks <= 1250) {
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

                println(it.dropTimer.isFinished)

                moveClaw && it.dropTimer.isFinished
            }
            .setOnExit {
                driveMoveFinished = false
                println("Score End")
                Scheduler.add(retractFromBasket)
            }

        retractFromBasket
            .setOnEnter {
                arm.target = Arm.Position.Down
            }
            .setAction {
                telemetry.addData("Move", "Retract")
//                it.slideEndTimer.update(it).finished {
                    slides.mpMove(leftSlidesHeight)
                    if (abs(slides.ticks - leftSlidesHeight) <= 2500) {
                        if (pivotShouldMove) {
                            pivot.mpSetPosition(-200)
                            pivotShouldMove = false
                        }
                    }
//                }

                pivot.ticks <= 1600 && !pivotShouldMove
            }
            .setOnExit {
                pivotShouldMove = true
                slidesShouldMove = true
                it.slideEndTimer.reset()
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
                drivetrain.moveToPose(groundRightPosition, Time.fromSeconds(3.5))
            }
            .setAction {
                telemetry.addData("Move", "Ground Right")
                if (drivetrain.moveFinished && !driveMoveFinished) {
                    driveMoveFinished = true
                    slides.mpMove(leftSlidesHeight)
                }

                if (abs(slides.ticks - leftSlidesHeight) < 350 && driveMoveFinished) {
                    claw.open = false
                }

                driveMoveFinished && abs(slides.ticks - leftSlidesHeight) < 350 && it.hoverTimer.update(it).isFinished
            }
            .setOnExit {
                driveMoveFinished = false
                moveClaw = false
                Scheduler.add(scoreHighBasket)
            }

//        groundMiddle
//            .setOnEnter {
//                drivetrain.moveToPose(groundMiddlePosition, Time.fromSeconds(3.5))
//            }
//            .setAction {
//                telemetry.addData("Move", "Ground Middle")
//                if (drivetrain.moveFinished) {
//                    driveMoveFinished = true
//                    claw.open = false
//                }
//
//                driveMoveFinished && it.hoverTimer.update(it).isFinished
//            }
//            .setOnExit {
//                scoreHighBasket.state.dropTimer.reset()
//                scoreHighBasket.state.pivotTimer.reset()
//                driveMoveFinished = false
//                moveClaw = false
//                Scheduler.add(scoreHighBasket)
//            }
        groundMiddle
            .setOnEnter {
                println("Moving to Middle")
                drivetrain.moveToPose(groundMiddlePosition, Time.fromSeconds(3.5))
            }
            .setAction {
                telemetry.addData("Move", "Ground Right")
                if (drivetrain.moveFinished && !driveMoveFinished) {
                    driveMoveFinished = true
                    slides.mpMove(leftSlidesHeight - 2500)
                }

                if (abs(slides.ticks - (leftSlidesHeight - 2500)) < 800 && driveMoveFinished) {
                    claw.open = false
                }

                driveMoveFinished && !claw.open && it.hoverTimer.update(it).isFinished
            }
            .setOnExit {
                driveMoveFinished = false
                moveClaw = false
                Scheduler.add(scoreHighBasket)
            }


        var bringIn = false
        groundLeft
            .setOnEnter {
                println("Moving to Left")
                drivetrain.moveToPose(groundLeftPosition, Time.fromSeconds(3.5))
            }
            .setAction {
                telemetry.addData("Move", "Ground Left")
                if (drivetrain.moveFinished && !driveMoveFinished) {
                    driveMoveFinished = true
                    slides.mpMove(leftSlidesHeight + 1000)
                }

                if (abs(slides.ticks - (leftSlidesHeight + 1000)) < 1000 && driveMoveFinished) {
                    claw.open = false
                }

                driveMoveFinished && !claw.open && it.hoverTimer.update(it).isFinished
            }
            .setOnExit {
                driveMoveFinished = false
                moveClaw = false
                Scheduler.add(scoreHighBasket)
            }

        Scheduler.add(moveForward)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            odometrySystem.logPosition(telemetry)
            claw.log(telemetry)
            arm.log(telemetry)
            slides.log(telemetry)
            pivot.log(telemetry)
            telemetry.update()
        }

        Scheduler.clear()
    }
}