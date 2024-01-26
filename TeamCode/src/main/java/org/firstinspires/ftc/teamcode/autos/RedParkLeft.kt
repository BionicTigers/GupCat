package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.Command
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name = "RedParkLeft", group = "Autonomous")
class RedParkLeft : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val intake = Intake(hardwareMap)
        robot.pose = Pose(863.6, 310.0, 0.0)
        val forward = Pose(863.6, 1400.0, 0.0)
        val parkPoint = Pose(2895.6, 1400.0, 0.0)
//        val spin = Pose(2895.0, 1370.0, 180.0)

        val group = CommandGroup()
            .add(Command { intake.down() })
            .add(drivetrain.moveToPosition(forward))
//            .await(750)
            .add(drivetrain.moveToPosition(parkPoint))
//            .add(drivetrain.moveToPosition(spin))
            .build()
        Scheduler.add(group)

        robot.onStart {
            Scheduler.update()
            robot.update()
        }

        Scheduler.clear()
    }

}