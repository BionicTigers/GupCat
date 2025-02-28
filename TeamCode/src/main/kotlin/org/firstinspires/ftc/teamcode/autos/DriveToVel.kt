package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Pose

interface VelocityState : CommandState {
    @Editable
    val velocityPose: Pose

    companion object {
        fun default(): VelocityState {
            return object : VelocityState, CommandState by CommandState.default("Meow :3") {
                override val velocityPose = Pose(0, 1000, 0)
            }
        }
    }
}

@Autonomous(name = "SimpleATV")
class DriveToVel : LinearOpMode() {
    override fun runOpMode() {
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)

        Scheduler.addSystem(odometrySystem, drivetrain)

        drivetrain.disabled = true

        Scheduler.add(
            Command(VelocityState.default())
                .setAction {
                    drivetrain.accelerateToVelocity(it.velocityPose)
                    false
                }
        )

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
        }
    }
}