//package org.firstinspires.ftc.teamcode.autos
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
//import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand
//import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
//import org.firstinspires.ftc.teamcode.mechanisms.Pivot
//import org.firstinspires.ftc.teamcode.mechanisms.Slides
//import org.firstinspires.ftc.teamcode.motion.Drivetrain
//import org.firstinspires.ftc.teamcode.motion.OdometrySystem
//import org.firstinspires.ftc.teamcode.utils.Pose
//
//@Autonomous(name = "RedParkLeft", group = "Autonomous")
//class RedLeftPark : LinearOpMode() {
//    override fun runOpMode() {
//        Scheduler.clear()
//        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
//        val odometrySystem = OdometrySystem(hardwareMap)
//        val drivetrain = Drivetrain(hardwareMap, gamepadSystem, odometrySystem)
//        val pivot = Pivot(hardwareMap)
//        val slides = Slides(hardwareMap)
//
//        Scheduler.addSystem(odometrySystem, drivetrain)
//
//        val moveForward = statelessCommand()
//        val liftPivot = statelessCommand()
//        val liftSlides = statelessCommand()
//        val moveToAscend = statelessCommand()
//
//        moveForward
//            .setOnEnter { drivetrain.moveToPose(Pose(0.0,1200.0,90.0)) }
//            .setAction  { println(drivetrain.moveFinished); println(odometrySystem.pose); drivetrain.moveFinished }
//            .setOnExit  { Scheduler.add(liftPivot) }
//        liftPivot
//            .setOnEnter { pivot.pivotTicks = 1000 }
//            .setAction  { pivot.currentPosition == pivot.pivotTicks }
//            .setOnExit  { Scheduler.add(liftSlides) }
//        liftSlides
//            .setOnEnter { slides.targetPosition = 3000 }
//            .setAction  { slides.currentPosition == slides.targetPosition }
//            .setOnExit  { Scheduler.add(moveToAscend) }
//        moveToAscend
//            .setOnEnter { drivetrain.moveToPose(Pose(200.0,1200.0,0.0)) }
//            .setAction { println(drivetrain.moveFinished); println(odometrySystem.pose); drivetrain.moveFinished }
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
//    }
//}