package org.firstinspires.ftc.teamcode.utils.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name = "ServoTestOp", group = "Testing")
class ServoTestOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val servoTest = ServoTest(hardwareMap)
        val (gamepad1, _) = robot.getGamepads()

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart {
            servoTest.run()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onEnd {
            servoTest.stop()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart {
            servoTest.backward()
        }

        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onEnd {
            servoTest.stop()
        }

        waitForStart()

        while (opModeIsActive()) {
            robot.update()
        }
    }

}