package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Time
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.timedCommand

@Autonomous(name = "BlueParkRightDeposit", group = "Autonomous")
class BlueParkRightDeposit : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear() //Clears all commands from the scheduler to allow a new OpMode to run
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val intake = Intake(hardwareMap)
        robot.pose = Pose(863.6, 3347.0, 180.0)
        val forward = Pose(863.6, 2100.0, 130.0)
        val parkPoint = Pose(2895.6, 2287.0, 90.0)

        val group = CommandGroup()
            .add(drivetrain.moveToPosition(forward))
//            .await(750)
            .add(drivetrain.moveToPosition(parkPoint))
            .add(timedCommand({intake.reverse()}, Time.fromSeconds(2.0)))
            .build()
        Scheduler.add(group)

        robot.onStart {
            Scheduler.update()
            robot.update()
        }

        Scheduler.clear()
    }
}