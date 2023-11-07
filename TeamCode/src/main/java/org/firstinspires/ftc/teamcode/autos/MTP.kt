package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.Scheduler

@Autonomous(name="MTP")
class MTP : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)

        val cmd = drivetrain.moveToPosition(Pose(-300.0, 1800.0, 0.0))
        Scheduler.add(cmd)

        waitForStart()
        while (opModeIsActive()) {
            robot.update()
        }

        Scheduler.clear()
    }
}