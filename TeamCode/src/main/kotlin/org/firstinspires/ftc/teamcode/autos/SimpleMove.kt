package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.utils.Timer
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose
import io.github.bionictigers.axiom.utils.Time
import io.github.bionictigers.axiom.web.Editable
import io.github.bionictigers.axiom.web.Server

interface CommandMove : CommandState {
    @Editable
    val poseToMove: Pose

    companion object {
        fun default(): CommandMove {
            return object : CommandMove, CommandState by CommandState.default("Meow :3") {
                override val poseToMove = Pose(0, 1000, 0)
            }
        }
    }
}

@Autonomous(name = "SimpleMTP")
class SimpleMove : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)

        Scheduler.addSystem(odometrySystem, drivetrain)

        Persistents.reset()

        Scheduler.add(
            Command(CommandMove.default())
                .setOnEnter {
                    drivetrain.moveToPose(it.poseToMove)
                }
                .setAction {
                   false
                }
                .setOnExit {
//                    Scheduler.add(otherMove)
                }
        )

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            odometrySystem.log(telemetry)

            telemetry.update()
        }
    }
}