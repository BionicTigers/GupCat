package org.firstinspires.ftc.teamcode.pedro

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.PoseUpdater
import com.pedropathing.pathgen.MathFunctions
import com.pedropathing.pathgen.Vector
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.motion.CustomPedroLocalizer
import java.util.Arrays
import kotlin.math.pow

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 * You can adjust the max velocity the robot will hit on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous(name = "Forward Zero Power Acceleration Tuner", group = "Automatic Tuners")
class ForwardZeroPowerAccelerationTuner : OpMode() {
    private val accelerations = ArrayList<Double>()

    private lateinit var leftFront: DcMotorEx
    private lateinit var leftRear: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightRear: DcMotorEx
    private lateinit var motors: List<DcMotorEx>

    private var poseUpdater: PoseUpdater? = null

    private var previousVelocity = 0.0

    private var previousTimeNano: Long = 0

    private lateinit var telemetryA: Telemetry

    private var stopping = false
    private var end = false

    /**
     * This initializes the drive motors as well as the FTC Dashboard telemetry.
     */
    override fun init() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val localizer = CustomPedroLocalizer(hardwareMap)
        poseUpdater = PoseUpdater(hardwareMap, localizer, FConstants::class.java, LConstants::class.java)

        leftFront = hardwareMap.get(DcMotorEx::class.java, FollowerConstants.leftFrontMotorName)
        leftRear = hardwareMap.get(DcMotorEx::class.java, FollowerConstants.leftRearMotorName)
        rightRear = hardwareMap.get(DcMotorEx::class.java, FollowerConstants.rightRearMotorName)
        rightFront = hardwareMap.get(DcMotorEx::class.java, FollowerConstants.rightFrontMotorName)
        leftFront.setDirection(FollowerConstants.leftFrontMotorDirection)
        leftRear.setDirection(FollowerConstants.leftRearMotorDirection)
        rightFront.setDirection(FollowerConstants.rightFrontMotorDirection)
        rightRear.setDirection(FollowerConstants.rightRearMotorDirection)

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear)

        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        for (motor in motors) {
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        telemetryA.addLine("The robot will run forward until it reaches " + VELOCITY + " inches per second.")
        telemetryA.addLine("Then, it will cut power from the drivetrain and roll to a stop.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.addLine("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.")
        telemetryA.addLine("Press CROSS or A on game pad 1 to stop.")
        telemetryA.update()
    }

    /**
     * This starts the OpMode by setting the drive motors to run forward at full power.
     */
    override fun start() {
        leftFront!!.power = 1.0
        leftRear!!.power = 1.0
        rightFront!!.power = 1.0
        rightRear!!.power = 1.0
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.cross || gamepad1.a) {
            for (motor in motors!!) {
                motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                motor.power = 0.0
            }
            requestOpModeStop()
        }

        poseUpdater!!.update()
        val heading = Vector(1.0, poseUpdater!!.pose.heading)
        if (!end) {
            if (!stopping) {
                if (MathFunctions.dotProduct(poseUpdater!!.velocity, heading) > VELOCITY) {
                    previousVelocity = MathFunctions.dotProduct(poseUpdater!!.velocity, heading)
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    for (motor in motors!!) {
                        motor.power = 0.0
                    }
                }
            } else {
                val currentVelocity = MathFunctions.dotProduct(poseUpdater!!.velocity, heading)
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < FollowerConstants.pathEndVelocityConstraint) {
                    end = true
                }
            }
        } else {
            var average = 0.0
            for (acceleration in accelerations) {
                average += acceleration
            }
            average /= accelerations.size.toDouble()

            telemetryA!!.addData("forward zero power acceleration (deceleration):", average)
            telemetryA!!.update()
        }
    }

    companion object {
        var VELOCITY: Double = 30.0
    }
}