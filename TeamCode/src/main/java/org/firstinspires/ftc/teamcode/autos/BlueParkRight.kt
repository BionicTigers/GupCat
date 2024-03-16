package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name = "BlueParkRight", group = "Autonomous")
class BlueParkRight : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        robot.pose = Pose(863.6, 3347.0, 180.0)
        val forward = Pose(863.6, 2287.0, 180.0)
        val intermediate = Pose(2013.6, 2087.0, 180.0)
        val parkPoint = Pose(3000.0, 2287.0, 180.0)

        val group = CommandGroup()
            .add({ drivetrain.moveToPosition(forward) })
            .add({ drivetrain.moveToPosition(intermediate) })
            .add({ drivetrain.moveToPosition(parkPoint) })
            .build()
        Scheduler.add(group)

        robot.onStart {
            Scheduler.update()
            robot.update()
        }

        Scheduler.clear()
    }
}