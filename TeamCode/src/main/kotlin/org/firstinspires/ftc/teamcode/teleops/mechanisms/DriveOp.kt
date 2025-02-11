package org.firstinspires.ftc.teamcode.teleops.mechanisms

import android.health.connect.datatypes.units.Velocity
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Vector2

@TeleOp(name="DriveOp", group = "mechanisms")
class DriveOp : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val odometrySystem = OdometrySystem(hardwareMap)
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)
        val dash = FtcDashboard.getInstance()

        Scheduler.addSystem(odometrySystem, gamepadSystem, drivetrain)
        waitForStart()
        odometrySystem.reset()

        while (opModeIsActive()) {
            val (velocity, angVelocity) = odometrySystem.globalVelocity
            odometrySystem.logPosition(telemetry)
            dash.telemetry.addData("linear x", velocity.x)
            dash.telemetry.addData("linear y", velocity.y)
            dash.telemetry.addData("angular", angVelocity.degrees)
            dash.telemetry.update()
            telemetry.update()
            Scheduler.update()
        }
        Scheduler.clear()
    }
}