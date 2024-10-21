//package org.firstinspires.ftc.teamcode.teleop
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import org.firstinspires.ftc.teamcode.mechanisms.Arm
//import org.firstinspires.ftc.teamcode.utils.  t
//import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
//
//@TeleOp(name = "ArmOp", group = "mechanisms")
//class ArmOp : LinearOpMode() {
//
//    override fun runOpMode() {
//        val robot = Robot(this)
//        val (gamepad1, gamepad2) = robot.getGamepads()
//        val arm = Arm(hardwareMap)
//
//        gamepad1.getButton(GamepadEx.Buttons.DPAD_UP).onStart { arm.up() }
//        gamepad1.getButton(GamepadEx.Buttons.DPAD_DOWN).onStart { arm.down() }
//
//        robot.onStart { robot.update() }
//    }
//}