package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.utils.Robot

class IntakeOpBlue : LinearOpMode(){
    override fun runOpMode() {
        val robot = Robot(this)
        val (gamepad1, gamepad2) = robot.getGamepads()
        val intake = Intake(hardwareMap)

        waitForStart()

        if(intake.colorSensor.blue() >= 100 && intake.colorSensor.green() <= 50 && intake.colorSensor.red() <= 50) {
            intake.start()
        } else {
            intake.stop()
        }

    }
}