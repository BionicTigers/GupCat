package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Chainbar
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
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
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        //Object declarations
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val output = Output(hardwareMap)
        val slides = Slide(hardwareMap)
        val chainbar = Chainbar(hardwareMap)
        val arm = Arm(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            hashMapOf("Red" to VisionConstants.RED))

        //Sets the robot's starting position
        robot.pose = Pose(902.0, 300.0, 180.0)

        //Creates potential scoring positions for the purple pixel on the spike marks
        val leftSpikeScore = Pose(622.0, 1010.0, 180.0)
        val middleSpikeScore = Pose(902.0, 1100.0, 180.0)
        val rightSpikeScore = Pose(1182.0, 1010.0, 180.0)

        val intermediate = Pose(902.0, 950.0, 180.0)
        val preTurn = Pose(902.0, 860.0, 180.0)
        val turn = Pose(902.0, 360.0, 270.0)
        val travel = Pose(2110.0, 360.0, 270.0)

        //Creates potential scoring positions for the purple pixel on the backdrop
        val leftBackdropScore = Pose(3225.0, 1130.0, 270.0)
        val middleBackdropScore = Pose(3225.0, 900.0, 270.0)
        val rightBackdropScore = Pose(3225.0, 810.0, 270.0)

        //Positions between backdrop scoring and parking
        val parkRight = Pose(3065.0, 345.0, 270.0)
        val preParkLeft = Pose(3065.0, 1615.0, 270.0)
        val parkLeft = Pose(3450.0, 1615.0, 270.0)

        val autoTime = ElapsedTime()
        var detection: Detection? = null

        //Create Commands
        val group1 = CommandGroup()
            .add(Command({
                val result = openCv.getDetection()
                RobotLog.ii("Contour x: ", result?.position?.x.toString())
                detection = when (result?.position?.x?.toInt()) {
                    in 1140..1280 -> Detection.Right
                    in 520..1140 -> Detection.Center
                    in 0..520 -> Detection.Left
                    else -> null
                }
            }) {detection == null}) //Gets camera detection
            .add { drivetrain.moveToPosition(intermediate) }
            .add {
                RobotLog.ii("Detection: ", detection?.name)
                return@add when (detection) {
                    Detection.Left -> drivetrain.moveToPosition(leftSpikeScore)
                    Detection.Center -> drivetrain.moveToPosition(middleSpikeScore)
                    Detection.Right -> drivetrain.moveToPosition(rightSpikeScore)
                    else -> drivetrain.moveToPosition(middleSpikeScore)
                }
            } //Moves to correct spike scoring position
            .add { drivetrain.moveToPosition(preTurn) }
            .add(timedCommand({ drivetrain.stop() }, Time.fromSeconds(1.0)))
            .add { drivetrain.moveToPosition(turn) }
            .add { drivetrain.moveToPosition(travel) }
            .add {
                return@add when (detection) {
                    Detection.Left -> drivetrain.moveToPosition(leftBackdropScore)
                    Detection.Center -> drivetrain.moveToPosition(middleBackdropScore)
                    Detection.Right -> drivetrain.moveToPosition(rightBackdropScore)
                    else -> drivetrain.moveToPosition(middleBackdropScore)
                }
            }
            .add(Command { slides.height = 1000.0 })
//            .add { drivetrain.moveToPosition(parkRight) }
            .add { drivetrain.moveToPosition(preParkLeft) }
            .add { drivetrain.moveToPosition(parkLeft) }
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