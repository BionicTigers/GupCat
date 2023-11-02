package org.firstinspires.ftc.teamcode.teleop
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.mechanisms.Marble
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name = "MarbleOp")
class MarbleOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (_, gamepad2) = robot.getGamepads()
        val marble = Marble(hardwareMap)

        gamepad2.getButton(GamepadEx.Buttons.LEFT_BUMPER).onStart {
            marble.move()
        }
        gamepad2.getButton(GamepadEx.Buttons.RIGHT_BUMPER).onStart {
            marble.stop()
        }

        waitForStart()

        while(opModeIsActive()) {
            robot.update()
        }
    }

}