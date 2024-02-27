package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Chainbar
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.command.Command
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.vision.VisionConstants

@Autonomous(name = "BluePreloadRight")
class BluePreloadRight : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        // Object declarations
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val output = Output(hardwareMap)
        val slides = Slide(hardwareMap)
        val chainbar = Chainbar(hardwareMap)
        val arm = Arm(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            hashMapOf("Blue" to VisionConstants.BLUE))

        // Sets the robot's starting position
        robot.pose = Pose(902.0, 286.0, 180.0)

        // Creates potential scoring positions for the purple pixel on the spike marks
        val leftSpikeScore = Pose(1133.0, 829.0, 180.0)
        val middleSpikeScore = Pose(902.0, 896.0, 180.0)
        val rightSpikeScore = Pose(658.0, 829.0, 180.0)

        // Creates potential scoring positions for the yellow pixel on the backdrop
        val leftBackdropScore = Pose(3060.0, 676.0, 90.0)
        val middleBackdropScore = Pose(3060.0, 932.0, 90.0)
        val rightBackdropScore = Pose(3060.0, 1109.0, 90.0)

        // Positions between backdrop scoring and parking
        val prePark = Pose(3060.0, 249.0, 90.0)
        val park = Pose(3352.0, 249.0, 90.0)

        // Creates variables used to represent detections
        val autoTime = ElapsedTime()
        var detection: Detection? = null

        //Create Commands
        val getDetection = Command({
            val result = openCv.getDetection()
            detection = when (result?.position?.x?.toInt()) {
                in 0..(1280 / 3) -> Detection.Left
                in (1280 / 3)..(1280 / 3 * 2) -> Detection.Center
                in (1280 / 3 * 2)..1280 -> Detection.Right
                else -> null
            }
        }) {return@Command detection == null}

        fun moveToSpike(): Command {
            return when (detection) {
                Detection.Left -> drivetrain.moveToPosition(leftSpikeScore)
                Detection.Center -> drivetrain.moveToPosition(middleSpikeScore)
                Detection.Right -> drivetrain.moveToPosition(rightSpikeScore)
                else -> drivetrain.moveToPosition(middleSpikeScore)
            }
        }

        fun moveToBackdrop(): Command {
            return when (detection) {
                Detection.Left -> drivetrain.moveToPosition(leftBackdropScore)
                Detection.Center -> drivetrain.moveToPosition(middleBackdropScore)
                Detection.Right -> drivetrain.moveToPosition(rightBackdropScore)
                else -> drivetrain.moveToPosition(middleBackdropScore)
            }
        }

        val preParkCommand = drivetrain.moveToPosition(prePark)
        val parkCommand = drivetrain.moveToPosition(park)

        val group1 = CommandGroup()
            .add(getDetection) //Gets camera detection
            .add(moveToSpike()) //Moves to correct spike scoring position
            .add(moveToBackdrop()) //Moves to correct backdrop scoring position
            .add(Command { chainbar.down() }) //Raises slides
            .add(Command { arm.up() })
            .add(Command { output.open() }) //Opens the right side of the output
            .add(preParkCommand) //Moves to the pre-parking position
            .add(parkCommand) //Moves to park position
            .build() //Builds all commands

        Scheduler.add(continuousCommand { slides.update() })
        Scheduler.add(group1)
        waitForStart()
        autoTime.reset()
        while(opModeIsActive()) {
            robot.update()
        }
    }
}