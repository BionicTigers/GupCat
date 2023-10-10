package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
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

    val visionPortal = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        .addProcessor(aprilTagProcessor)
        .setCameraResolution(android.util.Size(640,400))
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .enableCameraMonitoring(true)
        .setAutoStopLiveView(true)
        .build()


     override fun runOpMode() {

    }

}

//// Enable or disable the AprilTag processor.
//myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);