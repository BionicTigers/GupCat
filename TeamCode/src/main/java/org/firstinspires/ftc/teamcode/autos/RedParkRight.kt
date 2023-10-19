package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot

@Autonomous(name = "RedParkRight")
class RedParkRight : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        robot.pose = Pose(1517.0, 3340.0, 0.0)
        RedPark(robot, hardwareMap)
    }

}