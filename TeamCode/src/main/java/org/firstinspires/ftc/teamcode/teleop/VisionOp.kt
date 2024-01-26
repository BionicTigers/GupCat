package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.vision.Color
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.utils.vision.VisionConstants
import org.opencv.core.Scalar

@TeleOp(name = "VisionOp")
class VisionOp : LinearOpMode(){
    override fun runOpMode() {

        val signals = hashMapOf<String, Color>(
//            "Red" to Color(Scalar(0.0, 120.0, 70.0), Scalar(20.0, 200.0, 180.0), 1000),
//            "Blue" to Color(Scalar(80.0, 50.0, 20.0), Scalar(140.0, 250.0, 250.0), 1000),
            "Red" to VisionConstants.RED,
//            "Blue" to VisionConstants.BLUE,
        )

        val openCv = OpenCv(
            hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            signals,
            hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        )

        while (opModeInInit()) {
            telemetry.addData("Reading", openCv.getDetection()?.name ?: "None")
            telemetry.addData("x", openCv.getDetection()?.position?.x ?: "None")
            telemetry.addData("y", openCv.getDetection()?.position?.y ?: "None")
            telemetry.update()
        }
    }
}