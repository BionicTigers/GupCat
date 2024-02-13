package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name = "RedParkRight", group = "Autonomous")
class RedParkRight : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val intake = Intake(hardwareMap)
//        val wiggle = Pose(1517.0, 3435.0, 0.0)
        val parkPoint = Pose(3200.0, 315.0, 180.0)
        robot.pose = Pose(2082.0, 315.0, 180.0)

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