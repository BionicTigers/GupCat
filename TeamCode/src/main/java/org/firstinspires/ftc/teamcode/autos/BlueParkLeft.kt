package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name = "BlueParkLeft", group = "Autonomous")
class BlueParkLeft : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val wiggle = Pose(1504.0, 310.0, 0.0)
        val parkPoint = Pose(329.0, 310.0, 0.0)
        robot.pose = Pose(1499.0, 310.0, 0.0)
        val group = CommandGroup()
            .add(drivetrain.moveToPosition(wiggle))
            .await(750)
            .add(drivetrain.moveToPosition(parkPoint))
            .build()
        Scheduler.add(group)

        robot.onStart {
            Scheduler.update()
            robot.update()
        }

        Scheduler.clear()
    }
}