package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.utils.getByName

@TeleOp(name = "Test")
class Test : LinearOpMode() {
    override fun runOpMode() {
        val motor: DcMotorEx = hardwareMap.getByName("Motor")

        waitForStart()
        while (opModeIsActive()) {
            motor.power = .8
        }
    }
}