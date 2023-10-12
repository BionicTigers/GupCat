package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.tfod.TfodProcessor
import org.opencv.core.Size

@TeleOp(name = "AprilTagOp")
class AprilTagOp : LinearOpMode() {
    // Set custom features of the AprilTag Processor (Optional)
    val aprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawTagID(true)          // Default is true
        .setDrawTagOutline(true)     // Default is true
        .setDrawAxes(true)           // Default is false
        .setDrawCubeProjection(true) // Default is false
        .build()

    override fun runOpMode() {
        // Set custom features of the Vision Portal (Optional)
        val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(aprilTagProcessor)
            .enableCameraMonitoring(true)         // Enable LiveView (RC preview)
            .setAutoStopLiveView(true)            // Automatically stop LiveView (RC preview) when all vision processors are disabled.
            .build()

        waitForStart()

        while (opModeIsActive()) {
            aprilTagTelemetry()
            telemetry.update()


        }
    }

    private fun aprilTagTelemetry() {
        val currentDetections : List<AprilTagDetection> = aprilTagProcessor.getDetections()
        telemetry.addData(currentDetections.size.toString(), " Apriltags Detected")

        for (currentDetections in aprilTagProcessor.getDetections()) {
            if (currentDetections.metadata != null) {
                telemetry.addLine(
                    "ID: ${currentDetections.id} Name: ${currentDetections.metadata.name}"
                )
                telemetry.addLine(
                    "X: ${currentDetections.ftcPose.x} Y: ${currentDetections.ftcPose.y}"
                            )

                telemetry.addLine(
                    "Range: ${currentDetections.ftcPose.range} Bearing: ${currentDetections.ftcPose.bearing} Elevation: ${currentDetections.ftcPose.elevation} (inch, deg, deg)"
                ) /* Range, direct (point-to-point) distance to the tag center
                Bearing, the angle the camera must turn (left/right) to point directly at the tag center
                Elevation, the angle the camera must tilt (up/down) to point directly at the tag center
                */
            } else {
                telemetry.addLine("no apriltag :(")
            }
        }
    }



}

//// Enable or disable the AprilTag processor.
//myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);