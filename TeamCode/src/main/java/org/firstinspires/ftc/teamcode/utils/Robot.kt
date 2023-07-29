package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import org.firstinspires.ftc.teamcode.utils.movement.Odometry

class Robot(opMode: LinearOpMode) {
    val pose: Pose
        get() {
            return odometry.globalPose
        }

    private val odometry: Odometry = Odometry(opMode.hardwareMap)

    private val gamepad1: GamepadEx = GamepadEx(opMode.gamepad1)
    private val gamepad2: GamepadEx = GamepadEx(opMode.gamepad2)

    init {
//        Scheduler.add(ContinuousCommand
//        {
//            odometry.update()
//        }, 0)

        Scheduler.add(ContinuousCommand
        {
            gamepad1.update()
            gamepad2.update()
        })
    }

    fun getGamepads(): Pair<GamepadEx, GamepadEx> {
        return Pair(gamepad1, gamepad2)
    }

    fun update() {
        Scheduler.update()
    }
}