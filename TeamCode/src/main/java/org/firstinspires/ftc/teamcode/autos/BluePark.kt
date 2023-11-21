package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

/**
 * Starts in the rightmost position on the blue side of the field and drives to a position inside
 * the backstage area
 */

fun BluePark(robot: Robot, hardwareMap: HardwareMap) {
    val drivetrain = Drivetrain(hardwareMap, robot)
    val parkPoint = Pose(329.0, 310.0, 0.0)

    robot.onStart {
        Scheduler.add(drivetrain.moveToPosition(parkPoint))
        robot.update()
    }
}