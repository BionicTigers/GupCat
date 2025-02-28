package org.firstinspires.ftc.teamcode.teleops.mechanisms

import android.health.connect.datatypes.units.Velocity
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.input.GamepadSystem
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

        Scheduler.addSystem(odometrySystem, gamepadSystem, drivetrain)
        waitForStart()
        odometrySystem.reset()

        while (opModeIsActive()) {
            val (velocity, angVelocity) = odometrySystem.globalVelocity
            odometrySystem.logPosition(telemetry)
            telemetry.update()
            Scheduler.update()
        }
        Scheduler.clear()
    }
}