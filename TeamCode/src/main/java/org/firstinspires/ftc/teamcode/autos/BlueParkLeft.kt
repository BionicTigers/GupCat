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
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        robot.pose = Pose(2082.0, 3347.0, 180.0)


        val parkPoint = Pose(3200.0, 3347.0, 180.0)

        val group = CommandGroup()
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