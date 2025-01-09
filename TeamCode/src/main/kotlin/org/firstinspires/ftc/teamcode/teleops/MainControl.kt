package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Pose

@TeleOp(name = "MainControl")
class MainControl : LinearOpMode() {
    override fun runOpMode() {
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometry = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometry)
        val pivot = Pivot(hardwareMap)
        val slides = Slides(hardwareMap, pivot)
        val arm = Arm(hardwareMap)
        val claw = Claw(hardwareMap)

        odometry.globalPose = Pose(850.9, 215.9, 0)

        Scheduler.addSystem(gamepadSystem, odometry, drivetrain, slides, pivot)

        val (gp1, gp2) = gamepadSystem.gamepads

        slides.setupDriverControl(gp2) // dpad up and down
        pivot.setupDriverControl(gp2) // left trigger down, right trigger up
        arm.setupDriverControl(gp2) // left bumper up, right bumper down
        claw.setupDriverControl(gp2) // a toggles open and close

        val init = statelessCommand()
            .setOnEnter {
                claw.open = true
                arm.target = Arm.Position.Down
            }
            .setAction {
                true
            }

        Scheduler.add(init)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            odometry.log(telemetry)
            pivot.log(telemetry)
            arm.log(telemetry)
            slides.log(telemetry)
            telemetry.update()
        }

        Scheduler.clear()
    }
}