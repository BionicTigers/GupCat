package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

class Robot(opMode: LinearOpMode) {
    var pose: Pose = Pose()

    private val odometry: Odometry = Odometry()

    private val gamepad1: GamepadEx = GamepadEx(opMode.gamepad1)
    private val gamepad2: GamepadEx = GamepadEx(opMode.gamepad2)

    init {
        Scheduler.add(ContinuousCommand
        {
            gamepad1.update()
            gamepad2.update()
        }, 1)

        Scheduler.add(ContinuousCommand
        {
            odometry.update()
            pose = odometry.globalPose
        }, 2)
    }

    fun getGamepads(): Pair<GamepadEx, GamepadEx> {
        return Pair(gamepad1, gamepad2)
    }
}