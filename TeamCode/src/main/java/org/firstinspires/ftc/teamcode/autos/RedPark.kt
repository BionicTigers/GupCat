package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

fun RedPark (robot: Robot, hardwareMap: HardwareMap) {
    val drivetrain = Drivetrain(hardwareMap, robot)
    val parkPoint = Pose(3340.0, 304.0, 90.0)

    robot.onStart {
        Scheduler.add(drivetrain.moveToPosition(parkPoint))
        robot.update()
    }
}