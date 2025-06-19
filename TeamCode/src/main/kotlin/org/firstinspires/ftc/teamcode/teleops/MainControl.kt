package org.firstinspires.ftc.teamcode.teleops

import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.PoseUpdater
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import io.github.bionictigers.axiom.utils.Timer
import org.firstinspires.ftc.teamcode.input.Gamepad
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.localization.CustomPedroLocalizer
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.pedro.FConstants
import org.firstinspires.ftc.teamcode.pedro.LConstants
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose

@TeleOp(name = "MainControl")
class MainControl : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()

        val localizer = CustomPedroLocalizer(hardwareMap)
        val poseUpdater = PoseUpdater(hardwareMap, localizer, FConstants::class.java, LConstants::class.java)
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val slides = Slides(hardwareMap)
        val pivot = Pivot(hardwareMap, slides)
        slides.pivot = pivot
        val arm = Arm(hardwareMap)
        val claw = Claw(hardwareMap)
//        val timer = Timer(Time.fromSeconds(.25))

//        odometry.globalPose = Pose(850.9, 215.9, 0)

        Scheduler.addSystem(gamepadSystem, slides, pivot)

        val (gp1, gp2) = gamepadSystem.gamepads

        drivetrain.setupDriverControl(gp1) // b resets odo
//        Persistents.setupDriverControl(gp1) // y resets

        slides.setupDriverControl(gp2) // dpad up and down
        pivot.setupDriverControl(gp2) // left trigger down, right trigger up
        arm.setupDriverControl(gp2) // b toggles 180 degrees, y goes to 90
        claw.setupDriverControl(gp2) // a toggles open and close

        gp2.getBooleanButton(Gamepad.Buttons.LEFT_BUMPER).onDown {
            pivot.mpSetPosition(pivot.max)
            slides.mpMove(slides.max)
        }

        gp2.getBooleanButton(Gamepad.Buttons.RIGHT_BUMPER).onDown {
            slides.mpMove(0)
            pivot.mpSetPosition(0)
        }

        Scheduler.add(statelessCommand("initial")
            .setOnEnter {
                claw.open = true
                arm.target = Arm.Position.Down
                pivot.limitSwitch.state = true
            }
            .setAction {
                true
            }
        )

        Scheduler.add(Command)

        waitForStart()

        while (opModeIsActive()) {
            Persistents.log(telemetry)
            Scheduler.update()
//            odometry.logPosition(telemetry)
            pivot.log(telemetry)
//            arm.log(telemetry)
            slides.log(telemetry)
            telemetry.update()
        }

        Scheduler.clear()
    }
}