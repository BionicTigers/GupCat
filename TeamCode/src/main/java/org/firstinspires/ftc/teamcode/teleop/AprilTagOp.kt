package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.vision.AprilTags
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

@TeleOp(name = "AprilTagOp")
class AprilTagOp : LinearOpMode() {
    val aprilTags = AprilTags(hardwareMap)

    override fun runOpMode() {


        waitForStart()

        while (opModeIsActive()) {
            aprilTags.aprilTagTelemetry(telemetry)
            telemetry.update()
            aprilTags.calculateRobotPos(telemetry)
        }
    }

    }
}

//// Enable or disable the AprilTag processor.
//myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);