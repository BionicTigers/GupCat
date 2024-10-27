package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Arm

@TeleOp(name="DriveOp", group = "mechanisms")
class ArmOp : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val arm = Arm(hardwareMap)


        Scheduler.addSystem(gamepadSystem)
        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
        }
        Scheduler.clear()
    }
}
