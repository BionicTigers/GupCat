package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Persistents
import org.firstinspires.ftc.teamcode.utils.Pose

@Autonomous(name = "BlueParkRight", group = "Autonomous")
class BlueRightPark : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
        val odometrySystem = OdometrySystem(hardwareMap)
        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)

        Scheduler.addSystem(odometrySystem, drivetrain)

        Persistents.reset()

        val moveLeft = statelessCommand()

        moveLeft
            .setOnEnter { drivetrain.moveToPose(Pose(1200.0,50.0,0.0)) }
            .setAction { println(drivetrain.moveFinished); println(odometrySystem.globalPose); drivetrain.moveFinished }
//            .setOnExit { Scheduler.add(moveBack) }

        Scheduler.add(moveLeft)

        waitForStart()
        while (opModeIsActive()) {
            odometrySystem.log(telemetry)
            Scheduler.update()
        }
        Scheduler.clear()
    }
}