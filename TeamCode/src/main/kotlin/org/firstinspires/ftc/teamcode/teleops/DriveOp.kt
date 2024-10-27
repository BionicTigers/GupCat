package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.motion.OdometrySystem

@TeleOp(name="DriveOp", group = "mechanisms")
class DriveOp : LinearOpMode() {
    override fun runOpMode() {
        val odometrySystem = OdometrySystem(hardwareMap)
        Scheduler.addSystem(odometrySystem)
        odometrySystem.reset()
        waitForStart()
        odometrySystem.reset()


        while (opModeIsActive()) {
            odometrySystem.log(telemetry)
            Scheduler.update()
        }
        Scheduler.clear()
    }
}