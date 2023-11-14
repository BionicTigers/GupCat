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

@TeleOp(name = "RedPreloadRight")
class RedPreloadRight : LinearOpMode() {
    override fun runOpMode() {
        //Object declarations
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val output = Output(hardwareMap)
        val slides = Slide(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "webcam"),
            hashMapOf("red" to Color(Scalar(0.0, 255.0, 255.0), Scalar(0.0, 255.0, 255.0), 50)))

        //Sets the robot's starting position
        robot.pose = Pose(1505.0, 3352.0, 0.0)

        //Creates potential scoring positions for the yellow pixel on the spike marks */
        val leftSpikeScore = Pose(1810.0, 2816.0, 0.0)
        val middleSpikeScore = Pose(1505.0, 2706.0, 0.0)
        val rightSpikeScore = Pose(1243.0, 2816.0, 0.0)

        //Creates potential scoring positions for the purple pixel on the backdrop */
        val leftBackdropScore = Pose(750.0, 2456.0, 90.0)
        val middleBackdropScore = Pose(750.0, 2718.0, 90.0)
        val rightBackdropScore = Pose(750.0, 3023.0, 90.0)

        //Positions between backdrop scoring and parking
        val prePark = Pose(750.0, 3328.0, 90.0)
        val park = Pose(304.0, 3328.0, 90.0)

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
            // Gets camera detection
            .add(getDetection)
            .await(getDetection)
            .add(moveToSpike())
            .await(moveToSpike())
            .add(moveToBackdrop())
            .await(moveToBackdrop())
            .add(OnceCommand { slides.height = 400.0 })
            .await(400)
            .add(OnceCommand { output.openRight() })
            .await(200)
            .add(preParkCommand)
            .await(preParkCommand)
            .add(parkCommand)
            .build()

        Scheduler.add(ContinuousCommand { slides.update() })
        Scheduler.add(group1)
        waitForStart()
        autoTime.reset()
        while(opModeIsActive()) {
            robot.update()
        }
    }
}