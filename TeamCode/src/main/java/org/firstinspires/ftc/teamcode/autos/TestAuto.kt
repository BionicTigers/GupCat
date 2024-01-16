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

        val group = CommandGroup()
//            .add(drivetrain.moveToPosition(middleSpikeScore))
            .add(OnceCommand { println("hi") })
            .await(5)
            .add(OnceCommand { println("hi2") })
            .build()
        Scheduler.add(group)

        robot.onStart {
            Scheduler.update()
        }

        Scheduler.clear()    }

}