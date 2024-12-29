package org.firstinspires.ftc.teamcode.teleops.mechanisms

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.Camera
import org.firstinspires.ftc.teamcode.vision.OpenCV

@TeleOp(name = "VisionOp", group = "mechanisms")
class VisionOp : LinearOpMode() {
    override fun runOpMode() {
        val openCV = OpenCV(
            Camera(
                hardwareMap.get(WebcamName::class.java, "Webcam 1"),
                hardwareMap.appContext.resources.getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    hardwareMap.appContext.packageName
                )
            )
        )

        while (opModeInInit()) {
            telemetry.update()
        }
    }
}