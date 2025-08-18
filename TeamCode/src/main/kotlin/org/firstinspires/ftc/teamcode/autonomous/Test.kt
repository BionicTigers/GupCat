package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.utils.Pose

@Autonomous(name = "Sample")
class Test : LinearOpMode() {
    override fun runOpMode() {
        val odo = Odometry(hardwareMap, telemetry, Pose(0.0, 0.0, 0.0))
        val drivetrain = Drivetrain(hardwareMap, telemetry, odo)

        val move = drivetrain.moveToPosition(Pose(10.0, 10.0, 0.0))

        waitForStart()

        Scheduler.schedule(move)

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }
}