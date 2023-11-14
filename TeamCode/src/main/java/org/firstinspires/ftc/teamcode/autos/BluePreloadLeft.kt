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
import org.firstinspires.ftc.teamcode.utils.command.OnceCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.opencv.core.Scalar

@TeleOp(name = "BluePreloadLeft")
class BluePreloadLeft : LinearOpMode() {
    override fun runOpMode() {
        /** Object declarations */
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val output = Output(hardwareMap)
        val slides = Slide(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "webcam"),
            hashMapOf("blue" to Color(Scalar(239.0, 74.0, 66.0), Scalar(239.0, 74.0, 66.0), 50)))
        /** Sets the robot's starting position */
        robot.pose = Pose(2773.0, 3340.0, 0.0)
        /** Creates potential scoring positions for the yellow pixel on the spike marks */
        val leftSpikeScore = Pose(1213.0, 780.0, 0.0)
        val middleSpikeScore = Pose(1499.0, 902.0, 0.0)
        val rightSpikeScore = Pose(1810.0, 780.0, 0.0)
        /** Creates potential scoring positions for the purple pixel on the backdrop */
        val leftBackdropScore = Pose(750.0, 1072.0, -90.0)
        val middleBackdropScore = Pose(750.0, 1261.0, -90.0)
        val rightBackdropScore = Pose(750.0, 1450.0, -90.0)
        /** Positions between backdrop scoring and parking */
        val prePark = Pose(750.0, 310.0, -90.0)
        val Park = Pose(329.0, 310.0, -90.0)
        /** Creates variables used to represent detections */
        var case1 = false
        var case2 = false
        var case3 = false
        val autoTime = ElapsedTime()
        var detection: String? = null
        val group1 = CommandGroup()
            /** Gets camera detection */
            .add(ConditionalCommand({openCv.getDetection()}) {return@ConditionalCommand detection != null || autoTime.seconds() >= 5})
            /** Set case variables according to which detection is read */
            .add(ConditionalCommand({case1 = true; case2 = false; case3 = false}) {openCv.getDetection()?.position?.x!! <= 500})
            .add(ConditionalCommand({case1 = false; case2 = true; case3 = false}) {openCv.getDetection()?.position?.x!! > 500 && openCv.getDetection()?.position?.x!! < 1000})
            .add(ConditionalCommand({case1 = false; case2 = false; case3 = true}) {openCv.getDetection()?.position?.x!! >= 1000})
            /** Uses case variables to decide which scoring position to move to */
            .add(ConditionalCommand({drivetrain.moveToPosition(leftSpikeScore)}) {case1})
            .add(ConditionalCommand({drivetrain.moveToPosition(middleSpikeScore)}) {case2})
            .add(ConditionalCommand({drivetrain.moveToPosition(rightSpikeScore)}) {case3})
            .add(OnceCommand { output.openLeft() })
            .add(OnceCommand { output.close() })
            /** Uses case variables to move from the first scoring position to the second */
            .add(ConditionalCommand({drivetrain.moveToPosition(leftBackdropScore)}) {case1})
            .add(ConditionalCommand({drivetrain.moveToPosition(middleBackdropScore)}) {case2})
            .add(ConditionalCommand({drivetrain.moveToPosition(rightBackdropScore)}) {case3})
            .add(OnceCommand { slides.height = 400.0 })
            .add(OnceCommand { slides.update() }) //Note to self: Do slides initialize automatically or do I need to run init method manually? Ask Alex Mon
            .add(OnceCommand { output.openRight() })
            .add(OnceCommand { drivetrain.moveToPosition(prePark) })
            .add(OnceCommand { drivetrain.moveToPosition(Park) })
            .build()
        Scheduler.add(group1)
        waitForStart()
        autoTime.reset()
        while(opModeIsActive()) {
            robot.update()
        }
    }
}