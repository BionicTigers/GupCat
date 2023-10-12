package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import org.firstinspires.ftc.teamcode.utils.movement.Odometry

class Robot(private val opMode: LinearOpMode) {
    var pose: Pose = Pose()
    val hardwareMap: HardwareMap = opMode.hardwareMap

    private val odometry: Odometry = Odometry(this)

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

    fun onStart(loop: () -> Unit) {
        opMode.waitForStart()
        while (opMode.opModeIsActive()) {
            loop()
        }
    }
}