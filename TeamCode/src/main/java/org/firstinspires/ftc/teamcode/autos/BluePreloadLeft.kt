package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.command.Command
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.vision.VisionConstants

@Autonomous(name = "BluePreloadLeft")
class BluePreloadLeft : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()

        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val slides = Slide(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            hashMapOf("Blue" to VisionConstants.BLUE))

        robot.pose = Pose(2110.0, 3357.6, 0.0)

        val spikePositions = hashMapOf(
            "left" to Pose(1780.0, 2647.6, 0.0),
            "middle" to Pose(2110.0, 2557.6, 0.0),
            "right" to Pose(2444.0, 2647.6, 0.0)
        )

        val backdropPositions = hashMapOf(
            "left" to Pose(3050.0, 2375.2, 90.0),
            "middle" to Pose(3050.0, 2515.2, 90.0),
            "right" to Pose(3050.0, 2725.2, 90.0)
        )

        val centerSpike = Pose(2110.0, 2707.6, 0.0)
        val turn = Pose(2110.0, 2797.6, 90.0)

        val park = Pose(3050.0, 3200.6, 90.0)

        var detection: String? = null

        fun getPropLocation(): Command {
            return Command({
                val detectionPosition = openCv.getDetection()?.position

                detection = when (detectionPosition?.x?.toInt()) {
                    in 0..550 -> "right"
                    in 550..1140 -> "center"
                    in 1140..1280 -> "left"
                    else -> null
                }
            }) {detection == null}
        }

        fun spikeMove(): Command {
            val spikePosition = spikePositions[detection]

            return CommandGroup()
                .add(drivetrain.moveToPosition(centerSpike))
                .add(drivetrain.moveToPosition(spikePosition!!))
                .build()
        }

        fun backdropMove(): Command {
            val backdropPosition = backdropPositions[detection]

            return CommandGroup()
                .add(drivetrain.moveToPosition(turn))
                .add(drivetrain.moveToPosition(backdropPosition!!))
                .build()
        }

        val group1 = CommandGroup()
            .add { getPropLocation() }
            .add { spikeMove() }
            .add { backdropMove() }
            .add(Command { slides.height = 1000.0 })
            .add { drivetrain.moveToPosition(park) }
            .build()

        Scheduler.add(continuousCommand { slides.update() })
        Scheduler.add(group1)

        robot.onStart {
            robot.update()
        }
    }
}