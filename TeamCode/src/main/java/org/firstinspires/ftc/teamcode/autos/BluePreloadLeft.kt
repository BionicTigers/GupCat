package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
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

@Autonomous(name = "BluePreloadLeft")
class BluePreloadLeft : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        //Object declarations
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val intake = Intake(hardwareMap)
        val output = Output(hardwareMap)
        val slides = Slide(hardwareMap)
        val chainbar = Chainbar(hardwareMap)
        val arm = Arm(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            hashMapOf("Blue" to VisionConstants.BLUE))

        //Sets the robot's starting position
        robot.pose = Pose(2082.0, 3347.0, 0.0)

        //Creates potential scoring positions for the purple pixel on the spike marks
        val leftSpikeScore = Pose(2318.0, 2670.0, 0.0)
        val middleSpikeScore = Pose(2082.0, 2570.0, 0.0)
        val rightSpikeScore = Pose(1986.0, 2609.0, 41.5)

        val intermediate = Pose(2082.0, 2787.0, 0.0)
        val turn = Pose(2082.0, 2700.0, -90.0)

       //Creates potential scoring positions for the yellow pixel on the backdrop
        val leftBackdropScore = Pose(3000.0, 2600.0, -90.0)
        val middleBackdropScore = Pose(3000.0, 2500.0, -90.0)
        val rightBackdropScore = Pose(3000.0, 2400.0, -90.0)

        //Positions between backdrop scoring and parking
        val prePark = Pose(3020.0, 2000.0, -90.0)
        val park = Pose(3347.0, 3347.0, -90.0)

        val autoTime = ElapsedTime()
        var detection: Detection? = null

        val group1 = CommandGroup()
            .add(Command({
                val result = openCv.getDetection()
                RobotLog.ii("Contour x: ", result?.position?.x.toString())
                detection = when (result?.position?.x?.toInt()) {
                    in 0..500 -> Detection.Left
                    in 500..(1280 / 3 * 2) -> Detection.Center
                    in (1280 / 3 * 2)..1280 -> Detection.Right
                    else -> null
                }
            }) {detection == null}) //Gets camera detection
            .add(drivetrain.moveToPosition(intermediate))
            .add {
                RobotLog.ii("Detection: ", detection?.name)
                return@add when (detection) {
                    Detection.Left -> drivetrain.moveToPosition(leftSpikeScore)
                    Detection.Center -> drivetrain.moveToPosition(middleSpikeScore)
                    Detection.Right -> drivetrain.moveToPosition(rightSpikeScore)
                    else -> drivetrain.moveToPosition(middleSpikeScore)
                }
            } //Moves to correct spike scoring position
            .add(timedCommand({ drivetrain.stop() }, Time.fromSeconds(1.0)))
            .add(drivetrain.moveToPosition(intermediate))
//            .add(timedCommand({ drivetrain.stop() }, Time.fromSeconds(1.0)))
//            .add(drivetrain.moveToPosition(turn))
//            .add {
//                return@add when (detection) {
//                    Detection.Left -> drivetrain.moveToPosition(leftBackdropScore)
//                    Detection.Center -> drivetrain.moveToPosition(middleBackdropScore)
//                    Detection.Right -> drivetrain.moveToPosition(rightBackdropScore)
//                    else -> drivetrain.moveToPosition(middleBackdropScore)
//                }
//            }
//            .add(drivetrain.moveToPosition(prePark))
//            .add(drivetrain.moveToPosition(park))
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