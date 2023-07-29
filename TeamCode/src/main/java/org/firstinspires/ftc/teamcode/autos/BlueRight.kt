package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.vision.Color
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.utils.vision.VisionConstants

@Autonomous(name = "BlueRight")
class BlueRight : LinearOpMode() {
    val signals = hashMapOf<String, Color>(
        "Orange" to VisionConstants.ORANGE,
        "Purple" to VisionConstants.PURPLE,
        "Green" to VisionConstants.GREEN
    )

    val directions = hashMapOf(
        "Orange" to Vector2(1.0, 0.0),
        "Purple" to Vector2(0.0, 0.0),
        "Green" to Vector2(-1.0, 0.0),
    )

    override fun runOpMode() {
        var robot = Robot(this)
        var drivetrain = Drivetrain(hardwareMap, robot)
        var detector = OpenCv(hardwareMap.get(WebcamName::class.java, "Webcam 1"), signals)

        var state = 0

        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.SECONDS)
        var detection: String? = null

        waitForStart()

        while (opModeIsActive()) {
            if (state == 0) {
                detection = detector.getDetection()
                detection?.let { state = 1 }
                elapsedTime.reset()
            } else if (state == 1) {
                drivetrain.robotDMP(Vector2(0.0, -1.0))
                Thread.sleep(1000)
                state = 2
            } else if (state == 2) {
                directions[detection]?.let { drivetrain.robotDMP(it) }
                Thread.sleep(1000)
                state = 3
            }
        }

        Scheduler.clear()
    }
}

