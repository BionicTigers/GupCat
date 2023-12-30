package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.OnceCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name = "Test", group = "Autonomous")

class TestAuto : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val intake = Intake(hardwareMap)

        val middleSpikeScore = Pose(863.6, 919.0, 0.0)

        robot.pose = Pose(863.6, 310.0, 0.0)

        val group = CommandGroup()
//            .add(drivetrain.moveToPosition(middleSpikeScore))
            .add(OnceCommand { intake.down() })
            .add(OnceCommand { intake.startSlow() })
            .await(1)
            .add(OnceCommand { intake.stop() })
            .add(OnceCommand { intake.up() })
            .await(OnceCommand { intake.up() })
//            .add(drivetrain.moveToPosition())
            .build()
        Scheduler.add(group)

        robot.onStart {
            Scheduler.update()
            robot.update()
        }

        Scheduler.clear()    }

}