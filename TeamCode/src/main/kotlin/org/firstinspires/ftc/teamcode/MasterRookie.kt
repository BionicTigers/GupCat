package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.Drivetrain

class MasterRookie : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val drivetrain = Drivetrain(hardwareMap)

        Scheduler.addSystem(gamepadSystem, drivetrain)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
        }
    }

}