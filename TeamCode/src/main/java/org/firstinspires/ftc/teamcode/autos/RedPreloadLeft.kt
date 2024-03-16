package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Chainbar
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Time
import org.firstinspires.ftc.teamcode.utils.command.Command
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.command.timedCommand
import org.firstinspires.ftc.teamcode.utils.vision.VisionConstants

@Autonomous(name = "RedPreloadLeft")
class RedPreloadLeft : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()

        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val slides = Slide(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            hashMapOf("RED" to VisionConstants.RED))
        val output = Output(hardwareMap)
        val chainbar = Chainbar(hardwareMap)
        val intake = Intake(hardwareMap)
        val arm = Arm(hardwareMap)

        robot.pose = Pose(909, 3358, 0)

        val centerSpike = Pose(2127 - 1141, 2859, 0)

        val spikePositions = hashMapOf(
            "left" to Pose(2360 - 1141, 2624, -30),
            "middle" to Pose(2110 - 1141, 2521, 0),
            "right" to Pose(1940 - 1141, 2694, 45)
        )

        val backdropPositions = hashMapOf(
            "left" to Pose(1050 - 1141, 2590, 92),
            "middle" to Pose(1050 - 1141, 2759, 92),
            "right" to Pose(1050 - 1141, 2932, 92)
        )

        val preTurn = Pose(2110 - 1141, 3054, 0)
        val turn = Pose(-1515, 3054, 90)
        val park = Pose(1734 - 1141, 3340, 0)
        val gate = Pose(909, 1524, 0)

        var detection: String? = null

        fun getPropLocation(): Command {
            return timedCommand({
                val detectionPosition = openCv.getDetection()?.position

                detection = when (detectionPosition?.x?.toInt()) {
                    in 0..700 -> "middle"
                    in 700..1280 -> "right"
                    else -> "left"
                }
            }, Time(5.0))
        }

        fun spikeMove(): Command {
            val spikePosition = spikePositions[detection]

            return CommandGroup()
                .add(drivetrain.moveToPosition(centerSpike))
                .add { drivetrain.moveToPosition(spikePosition!!) }
                .build()
        }

        fun backdropMove(): Command {
            val spikePosition = spikePositions[detection]
            val backdropPosition = backdropPositions[detection]
            val localizedPreTurn = Pose(preTurn.x, preTurn.y, spikePosition!!.rotation)

            return CommandGroup()
                .add { drivetrain.moveToPosition(gate) }
                .add { drivetrain.moveToPosition(localizedPreTurn) }
                .add { drivetrain.moveToPosition(turn) }
                .add { drivetrain.moveToPosition(backdropPosition!!) }
                .build()
        }

        fun deposit(): Command {
            return CommandGroup()
                .add(Command { slides.height = 1000.0 })
                .add(Command({}) {slides.state != Slide.SlideState.Idle})
                .add(timedCommand({ arm.up() }, Time(1.0)))
                .add(Command { output.open() })
                .add(timedCommand({}, Time(2.0)))
                .build()
        }

        fun reset(): Command {
            return CommandGroup()
                .add(Command { arm.down() })
                .add(Command { slides.height = 0.0 })
                .build()
        }

        val group1 = CommandGroup()
            .add { getPropLocation() }
            .add { spikeMove() }
            .add { backdropMove() }
            .add { deposit() }
            .add { reset() }
            .add { drivetrain.moveToPosition(park) }
            .build()

        Scheduler.add(continuousCommand { slides.update() })
        Scheduler.add(group1)

        chainbar.down()
        intake.up()
        output.close()
        arm.down()

        robot.onStart {
            robot.update()
        }
    }
}

