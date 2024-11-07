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

@Autonomous(name = "BlueParkLeft", group = "Autonomous")
class BlueLeftPark : LinearOpMode() {
    override fun runOpMode() {
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)

        Scheduler.addSystem(odometrySystem, drivetrain)

//        val setMode = statelessCommand()
        val move = statelessCommand()

//        setMode
//            .setOnEnter { drivetrain.setMode(Drivetrain.ControlMode.AUTONOMOUS) }
//            .setAction { false }
//            .setOnExit { Scheduler.add(move) }

        move
            .setOnEnter {
                drivetrain.moveToPose(Pose(120.0,1500.0,0.0))
            }
            .setAction {
                println("moving")
                println(drivetrain.moveFinished)
//                drivetrain.moveFinished
                false
                }

        Scheduler.add(move)

        waitForStart()
        while (opModeIsActive()) {
            odometrySystem.log(telemetry)
            Scheduler.update()
        }
    }
}