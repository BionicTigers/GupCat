package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem

@TeleOp(name = "MainControl")
class MainControl : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()

        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometry = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometry)
        val pivot = Pivot(hardwareMap)
        val slides = Slides(hardwareMap, pivot)
        val arm = Arm(hardwareMap)
        val claw = Claw(hardwareMap, 0.0)

//        odometry.globalPose = Pose(850.9, 215.9, 0)

        Scheduler.addSystem(gamepadSystem, odometry, drivetrain, slides, pivot)

        val (gp1, gp2) = gamepadSystem.gamepads

        drivetrain.setupDriverControl(gp1) // b resets odo
//        Persistents.setupDriverControl(gp1) // y resets

        slides.setupDriverControl(gp2) // dpad up and down
        pivot.setupDriverControl(gp2) // left trigger down, right trigger up
        arm.setupDriverControl(gp2) // b toggles 180 degrees, y goes to 90
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
//            Persistents.log(telemetry)
            Scheduler.update()
            odometry.logMaximums(telemetry)
            pivot.log(telemetry)
            arm.log(telemetry)
            slides.log(telemetry)
            telemetry.update()
        }

        Scheduler.clear()
    }
}