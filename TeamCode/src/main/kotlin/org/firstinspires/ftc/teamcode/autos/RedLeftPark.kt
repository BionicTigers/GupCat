package org.firstinspires.ftc.teamcode.autos

//import io.github.bionictigers.axiom.commands.statelessCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Scheduler

@Autonomous(name = "RedParkLeft", group = "Autonomous")
class RedLeftPark : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.clear()
//        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
//        val odometrySystem = OdometrySystem(hardwareMap)
//        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)
//        val pivot = Pivot(hardwareMap)
////        val slides = Slides(hardwareMap)
//
//        Scheduler.addSystem(odometrySystem, drivetrain)
//
//        Persistents.reset()
//
//        val moveForward = statelessCommand()
//        val liftPivot = statelessCommand()
//        val liftSlides = statelessCommand()
//        val moveToAscend = statelessCommand()
//
//        moveForward
//            .setOnEnter { drivetrain.moveToPose(Pose(0.0,1200.0,90.0)) }
//            .setAction  { println(drivetrain.moveFinished); println(odometrySystem.globalPose); drivetrain.moveFinished }
//            .setOnExit  { Scheduler.add(liftPivot) }
//        liftPivot
//            .setOnEnter { pivot.pivotTicks = 1000 }
//            .setAction  { pivot.currentPosition == pivot.pivotTicks }
//            .setOnExit  { Scheduler.add(liftSlides) }
//        liftSlides
////            .setOnEnter { slides.targetPosition = 3000 }
////            .setAction  { slides. == slides.targetPosition }
//            .setOnExit  { Scheduler.add(moveToAscend) }
//        moveToAscend
//            .setOnEnter { drivetrain.moveToPose(Pose(200.0,1200.0,0.0)) }
//            .setAction { println(drivetrain.moveFinished); println(odometrySystem.globalPose); drivetrain.moveFinished }
//            .setOnExit { Scheduler.add() }
//
//        Scheduler.add(moveForward)
//
//        waitForStart()
//        while (opModeIsActive()) {
//            odometrySystem.log(telemetry)
//            Scheduler.update()
//        }
//        Scheduler.clear()
    }
}