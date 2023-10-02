package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mechanisms.Drone
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx

@TeleOp(name="LiftOp")

class DroneOp : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        val (_, gamepad2) = robot.getGamepads()
        val drone = Drone(hardwareMap)

        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart{
            drone.start()
        }
        gamepad2.getButton(GamepadEx.Buttons.DPAD_UP).onStart{
            drone.stop()
        }
    }
}