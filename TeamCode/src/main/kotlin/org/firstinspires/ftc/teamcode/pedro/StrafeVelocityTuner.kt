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
import kotlin.math.abs


/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot right at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 * You can adjust the distance the robot will travel on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous(name = "Strafe Velocity Tuner", group = "Automatic Tuners")
class StrafeVelocityTuner : OpMode() {
    private val velocities = ArrayList<Double>()

    private lateinit var leftFront: DcMotorEx
    private lateinit var leftRear: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightRear: DcMotorEx
    private lateinit var motors: List<DcMotorEx>

    private var poseUpdater: PoseUpdater? = null

    private lateinit var telemetryA: Telemetry

    private var end = false

    /**
     * This initializes the drive motors as well as the cache of velocities and the FTC Dashboard
     * telemetry.
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

        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }

        telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        telemetryA.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the right.")
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.")
        telemetryA.addLine("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.")
        telemetryA.addLine("Press CROSS or A on game pad 1 to stop.")
        telemetryA.update()
    }

    /**
     * This starts the OpMode by setting the drive motors to run right at full power.
     */
    override fun start() {
        leftFront!!.power = 1.0
        leftRear!!.power = -1.0
        rightFront!!.power = -1.0
        rightRear!!.power = 1.0
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
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
        if (!end) {
            if (abs(poseUpdater!!.pose.y) > DISTANCE) {
                end = true
                for (motor in motors!!) {
                    motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                    motor.power = 0.0
                }
            } else {
                val currentVelocity = abs(
                    MathFunctions.dotProduct(
                        poseUpdater!!.velocity, Vector(1.0, Math.PI / 2)
                    )
                )
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            leftFront!!.power = 0.0
            leftRear!!.power = 0.0
            rightFront!!.power = 0.0
            rightRear!!.power = 0.0
            for (motor in motors!!) {
                motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }
            var average = 0.0
            for (velocity in velocities) {
                average += velocity
            }
            average /= velocities.size.toDouble()

            telemetryA!!.addData("strafe velocity:", average)
            telemetryA!!.update()
        }
    }

    companion object {
        var DISTANCE: Double = 48.0
        var RECORD_NUMBER: Double = 10.0
    }
}