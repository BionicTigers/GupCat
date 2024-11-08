package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.random.Random

@Autonomous(name = "BlueParkRight", group = "Autonomous")
class BlueRightPark : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)

        Scheduler.addSystem(odometrySystem, drivetrain)

        val moveForward = statelessCommand()
        val moveLeft = statelessCommand()
        val moveBack = statelessCommand()

        moveForward
            .setOnEnter { drivetrain.moveToPose(Pose(0.0,600.0,0.0)) }
            .setAction { println(drivetrain.moveFinished); drivetrain.moveFinished }
            .setOnExit { Scheduler.add(moveLeft) }

        moveLeft
            .setOnEnter { drivetrain.moveToPose(Pose(600.0,600.0,0.0)) }
            .setAction { println(drivetrain.moveFinished); println(odometrySystem.pose); drivetrain.moveFinished }
            .setOnExit { Scheduler.add(moveBack) }

        moveBack
            .setOnEnter { drivetrain.moveToPose(Pose(600.0,50.0,0.0)) }
            .setAction { println(drivetrain.moveFinished); println(odometrySystem.pose); odometrySystem.log(telemetry); drivetrain.moveFinished }



        Scheduler.add(moveForward)

        waitForStart()
        while (opModeIsActive()) {
            odometrySystem.log(telemetry)
            Scheduler.update()
        }
        Scheduler.clear()
    }
}