package org.firstinspires.ftc.teamcode.teleop

import android.graphics.Rect
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.vision.Color
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.utils.vision.VisionConstants

@TeleOp(name = "VisionOp")
class VisionOp : LinearOpMode(){
    override fun runOpMode() {
        val signals = hashMapOf<String, Color>(
            "Orange" to VisionConstants.ORANGE,
            "Purple" to VisionConstants.PURPLE,
            "Green" to VisionConstants.GREEN
        )

        val openCv = OpenCv(
            hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            signals
        )
    }
}