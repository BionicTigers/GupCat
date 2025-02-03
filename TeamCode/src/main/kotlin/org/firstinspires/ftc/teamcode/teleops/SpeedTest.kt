package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.Time

@TeleOp(name = "Speed")
class SpeedTest : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()

        var allTimes = arrayListOf<Long>()
        var lastTime = System.nanoTime()
        while (opModeIsActive()) {
            allTimes.add(lastTime - System.nanoTime())
            lastTime = System.nanoTime()
        }
        val sum = allTimes.sum()
        println(Time.fromNanoseconds(sum / allTimes.size).milliseconds())
    }
}