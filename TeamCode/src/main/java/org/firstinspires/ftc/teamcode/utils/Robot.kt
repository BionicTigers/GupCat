package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import org.firstinspires.ftc.teamcode.utils.movement.Odometry

class Robot(private val opMode: LinearOpMode) {
    var pose: Pose = Pose()
    val hardwareMap: HardwareMap = opMode.hardwareMap

    val telemetry: Telemetry = opMode.telemetry

    private val odometry: Odometry = Odometry(this)

    private val gamepad1: GamepadEx = GamepadEx(opMode.gamepad1)
    private val gamepad2: GamepadEx = GamepadEx(opMode.gamepad2)

    init {
        Scheduler.add(continuousCommand
        {

            odometry.update()
            opMode.telemetry.addData("x", pose.x)
            opMode.telemetry.addData("y", pose.y)
            opMode.telemetry.addData("rot", pose.rotation)
            opMode.telemetry.update()
        })

        Scheduler.add(continuousCommand
        {
            gamepad1.update()
            gamepad2.update()
        })

//        val a = arrayListOf<Double>()
//        val b = ElapsedTime()
//        var j = 0.0
//        fun sum(): Double {
//            var total = 0.0
//            a.forEach {
//                total += it
//            }
//
//            return total
//        }
//        Scheduler.add(continuousCommand {
//            a.add((pose.y - j) / b.seconds())
//            j = pose.y
//            telemetry.addData("velo", sum() / a.size )
//            b.reset()
//        })
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