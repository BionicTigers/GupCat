package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.vision.Color
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.ConditionalCommand
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.OnceCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.opencv.core.Scalar

@TeleOp(name = "BluePreloadRight")
class BluePreloadRight : LinearOpMode() {
    override fun runOpMode() {
        // Object declarations 
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val output = Output(hardwareMap)
        val slides = Slide(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "webcam"),
            hashMapOf("blue" to Color(Scalar(239.0, 74.0, 66.0), Scalar(239.0, 74.0, 66.0), 50)))

        // Sets the robot's starting position 
        robot.pose = Pose(2761.0, 310.0, 0.0)

        // Creates potential scoring positions for the purple pixel on the spike marks
        val leftSpikeScore = Offsets["left"]!!
        val middleSpikeScore = Offsets["center"]!!
        val rightSpikeScore = Offsets["right"]!!

        // Creates potential scoring positions for the yellow pixel on the backdrop
        val leftBackdropScore = Pose(750.0, 1072.0, -90.0)
        val middleBackdropScore = Pose(750.0, 1261.0, -90.0)
        val rightBackdropScore = Pose(750.0, 1450.0, -90.0)

        // Positions between backdrop scoring and parking 
        val prePark = Pose(750.0, 310.0, -90.0)
        val park = Pose(329.0, 310.0, -90.0)

        // Creates variables used to represent detections
        val autoTime = ElapsedTime()
        var detection: Detection? = null

        //Create Commands
        val getDetection = ConditionalCommand({
            val result = openCv.getDetection()
            detection = when (result?.position?.x?.toInt()) {
                in 0..(1280 / 3) -> Detection.Left
                in (1280 / 3)..(1280 / 3 * 2) -> Detection.Center
                in (1280 / 3 * 2)..1280 -> Detection.Right
                else -> null
            }
        }) {return@ConditionalCommand detection == null || autoTime.seconds() >= 5}

        fun moveToSpike(): ConditionalCommand? {
            return when (detection) {
                Detection.Left -> drivetrain.moveToPosition(leftSpikeScore)
                Detection.Center -> drivetrain.moveToPosition(middleSpikeScore)
                Detection.Right -> drivetrain.moveToPosition(rightSpikeScore)
                else -> null
            }
        }

        fun moveToBackdrop(): ConditionalCommand? {
            return when (detection) {
                Detection.Left -> drivetrain.moveToPosition(leftBackdropScore)
                Detection.Center -> drivetrain.moveToPosition(middleBackdropScore)
                Detection.Right -> drivetrain.moveToPosition(rightBackdropScore)
                else -> null
            }
        }

        val preParkCommand = drivetrain.moveToPosition(prePark)
        val parkCommand = drivetrain.moveToPosition(park)

        val group1 = CommandGroup()
            .add(getDetection) //Gets camera detection
            .await(getDetection) //Waits for previous command to end
            .add(moveToSpike()) //Moves to correct spike scoring position
            .await(moveToSpike())
            .add(moveToBackdrop()) //Moves to correct backdrop scoring position
            .await(moveToBackdrop())
            .add(OnceCommand { slides.height = 400.0 }) //Raises slides
            .await(400) //Waits 400 ms
            .add(OnceCommand { output.open() }) //Opens the right side of the output
            .await(200) //Waits 200 ms
            .add(preParkCommand) //Moves to the pre-parking position
            .await(preParkCommand)
            .add(parkCommand) //Moves to park position
            .build() //Builds all commands

        Scheduler.add(ContinuousCommand { slides.update() })
        Scheduler.add(group1)
        waitForStart()
        autoTime.reset()
        while(opModeIsActive()) {
            robot.update()
        }
    }
}