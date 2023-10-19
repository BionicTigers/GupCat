package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Slide
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.vision.Color
import org.firstinspires.ftc.teamcode.utils.vision.OpenCv
import org.firstinspires.ftc.teamcode.autos.Offsets
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.command.ConditionalCommand
import org.opencv.core.Scalar

@TeleOp(name = "BluePreload")
class BluePreloadScore : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val output = Output(hardwareMap)
        val slides = Slide(hardwareMap)
        val openCv = OpenCv(hardwareMap.get(WebcamName::class.java, "webcam"),
            hashMapOf("blue" to Color(Scalar(6.0, 150.0, 130.0), Scalar(6.0, 150.0, 130.0), 50)))
        val offsets = Offsets()
        val case1 = offsets.map[0]
        val case2 = offsets.map[1]
        val case3 = offsets.map[2]
        var dropSpot = Pose()

    }
}