package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name = "RedParkRight", group = "Autonomous")
class RedParkRight : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
//        val wiggle = Pose(1517.0, 3435.0, 0.0)
        val parkPoint = Pose(2890.0, 315.0, 0.0)
        robot.pose = Pose(2082.0, 310.0, 0.0)

        val group = CommandGroup()
//            .add(drivetrain.moveToPosition(wiggle))
//            .await(750)
            .add(drivetrain.moveToPosition(parkPoint))
            .build()
        Scheduler.add(group)

        robot.onStart {
            robot.update()
        }

        Scheduler.clear()
    }

}