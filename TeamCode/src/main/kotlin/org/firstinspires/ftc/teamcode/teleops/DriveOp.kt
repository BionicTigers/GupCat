package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem

@TeleOp(name="DriveOp", group = "mechanisms")
class DriveOp : LinearOpMode() {
    override fun runOpMode() {
        val odometrySystem = OdometrySystem(hardwareMap)
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val drivetrainSystem = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)
        Scheduler.addSystem(odometrySystem)
        Scheduler.addSystem(drivetrainSystem)

        waitForStart()
        odometrySystem.reset()


        while (opModeIsActive()) {
            odometrySystem.log(telemetry)
            Scheduler.update()
        }
        Scheduler.clear()
    }
}