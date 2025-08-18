package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.System
import io.github.bionictigers.axiom.web.WebData
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.input.ControlSchema
import org.firstinspires.ftc.teamcode.input.Controllable
import org.firstinspires.ftc.teamcode.input.Controls
import org.firstinspires.ftc.teamcode.input.Gamepads
import org.firstinspires.ftc.teamcode.input.Profile
import org.firstinspires.ftc.teamcode.input.matches
import org.firstinspires.ftc.teamcode.input.types.Analog
import org.firstinspires.ftc.teamcode.motion.LQR
import org.firstinspires.ftc.teamcode.motion.MotionProfile
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.createTuningMatrix
import org.firstinspires.ftc.teamcode.motion.toMK2D
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.getByName
import org.jetbrains.kotlinx.multik.api.identity
import org.jetbrains.kotlinx.multik.api.mk
import org.jetbrains.kotlinx.multik.ndarray.data.D2Array
import org.jetbrains.kotlinx.multik.ndarray.operations.*
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.time.Duration

class Drivetrain(hardwareMap: HardwareMap, telemetry: Telemetry? = null, private val odometry: Odometry) : System, Controllable {
    enum class DriveOrientation {
        /** Movement is relative to the robot */
        ROBOT,
        /** Movement is relative to the field, direction is constant */
        FIELD
    }

    interface Schema : ControlSchema {
        /** The method used to control the robot */
        val orientation: DriveOrientation

        /** Lateral Movement (Left-Right) */
        val x: Analog

        /** Longitudinal Movement (Forward-Backward) */
        val y: Analog

        /** Rotational Movement */
        val rot: Analog
    }

    companion object {
        val xJerk = 6000.0
        val yJerk = 3000.0
        val angularJerk = Angle.degrees(900.0)
        val xMaxAcceleration = 3509.0
        val yMaxAcceleration = 3569.0/4.0
        val angularMaxAcceleration = Angle.degrees(900.0) //130
        val xMaxVelocity = 1003.0
        val yMaxVelocity = 1812.0
        val angularMaxVelocity = Angle.degrees(310.0)

        val xProfile = MotionProfile(xJerk, xMaxAcceleration, xMaxVelocity)
        val yProfile = MotionProfile(yJerk, yMaxAcceleration, yMaxVelocity)
        val angularProfile = MotionProfile(angularJerk.radians, angularMaxAcceleration.radians, angularMaxVelocity.radians)

        const val STALL_TORQUE = 1.83384355 // Nm
        const val FREE_SPEED = 45.553093 // rad/s
        const val WHEEL_RADIUS = 0.096 // m
        const val ROBOT_MASS = 13.33562 // kg
        const val LX = 0.1778 // horizontal distance from wheel to center of robot
        const val LY = 0.1444625 // vertical distance from wheel to center of robot

        val xCost = 1.0
        val yCost = 1.0
        val thetaCost = 1.0
        val vxCost = 1.0
        val vyCost = 1.0
        val omegaCost = 1.0

        val controlCost = 1.0
    }

    override val name = "drivetrain"

    private val motors = DriveMotors(
        "frontLeft",
        "backLeft",
        "frontRight",
        "backRight",
        hardwareMap
    )

    val data = DrivetrainData()

    val state = DrivetrainState()

    init {
        if (telemetry != null) {
            Scheduler.schedule(Command.continuous("Drivetrain Log") {
//                telemetry.addData("Drivetrain X", data.xControl)
//                telemetry.addData("Drivetrain Y", data.yControl)
//                telemetry.addData("Drivetrain Rot", data.rotControl)

                telemetry.addData("Odometry X", odometry.position.x)
                telemetry.addData("Odometry Y", odometry.position.y)
                telemetry.addData("Odometry Heading", odometry.position.radians)
                telemetry.addData("Odometry Velocity X", odometry.velocity.x)
                telemetry.addData("Odometry Velocity Y", odometry.velocity.y)
                telemetry.addData("Odometry Velocity Heading", odometry.velocity.radians)

            })
        }

        motors.forEach { motor ->
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private val jacobians = JacobianCalculations()
    private var q = createTuningMatrix(xCost, yCost, thetaCost, vxCost, vyCost, omegaCost)
    private var r = mk.identity<Double>(4) * controlCost
    private var lqr = LQR(jacobians.a, jacobians.b, q, r)

    override val dependencies = listOf(odometry)

    private val headingPID = PID(8.0, 0.0, 0.0, -PI, PI, -1.0, 1.0)
    private var targetHeading = 0.0
    private var timeLetGo = Duration.ZERO
    private var rotation = 0.0

    override val beforeRun = Command.continuous("Motor Power Calculation", data) {
        if (it.isInTeleop) {
            if (it.xControl.absoluteValue > .05) {
                targetHeading = odometry.position.radians % (2 * PI)
            }
            if (it.xControl.absoluteValue > .05) {
                rotation = -it.xControl
                timeLetGo = it.enteredAt?.elapsedNow() ?: Duration.ZERO
            } else {
                val shortestPath = atan2(
                    sin(odometry.position.radians - targetHeading),
                    cos(odometry.position.radians - targetHeading)
                )
                rotation = -headingPID.calculate(0.0, shortestPath) / PI
            }
        } else {
//            val k = lqr.computeK()
//            state.controlMatrix = -k * state.stateMatrix
        }
    }

    override val afterRun = Command.continuous("Drivetrain Update", data) {
        if (it.isInTeleop) {
            rotMulti = 1 - (it.xControl.absoluteValue.coerceAtLeast(it.yControl.absoluteValue) * .6)
        } else {
            motors.setPower(state.controlMatrix)
        }
    }

    fun moveToPosition(targetPosition: Pose) : Command<DrivetrainState> = Command.create("Move To Position", state) {
        lateinit var xMotionResult: MotionResult
        lateinit var yMotionResult: MotionResult
        lateinit var thetaMotionResult: MotionResult

        enter {
            xMotionResult = xProfile.generate(odometry.position.x, targetPosition.x)
            yMotionResult = yProfile.generate(odometry.position.y, targetPosition.y)
            thetaMotionResult = angularProfile.generate(odometry.position.radians, targetPosition.radians)
        }

        action {
            with(it) {
                x = xMotionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.position.x
                y = yMotionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.position.y
                theta = thetaMotionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.position.radians
                vx = xMotionResult.getVelocity(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.velocity.x
                vy = yMotionResult.getVelocity(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.velocity.y
                omega = thetaMotionResult.getVelocity(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.velocity.radians
                odometry.position == targetPosition
            }
        }

        exit { it.reset() }
    }

    fun moveToPosition(
        targetPosition: Pose,
        newXCost: Double,
        newYCost: Double,
        newThetaCost: Double,
        newVXCost: Double,
        newVYCost: Double,
        newOmegaCost: Double,
        newControlCost: Double
    ) : Command<DrivetrainState> = Command.create("Move To Position", state) {
        lateinit var xMotionResult: MotionResult
        lateinit var yMotionResult: MotionResult
        lateinit var thetaMotionResult: MotionResult

        enter {
            xMotionResult = xProfile.generate(odometry.position.x, targetPosition.x)
            yMotionResult = yProfile.generate(odometry.position.y, targetPosition.y)
            thetaMotionResult = angularProfile.generate(odometry.position.radians, targetPosition.radians)

            q = createTuningMatrix(newXCost, newYCost, newThetaCost, newVXCost, newVYCost, newOmegaCost)
            r = mk.identity<Double>(4) * newControlCost
            lqr = LQR(jacobians.a, jacobians.b, q, r)
        }

        action {
            with(it) {
                x = xMotionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.position.x
                y = yMotionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.position.y
                theta = thetaMotionResult.getPosition(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.position.radians
                vx = xMotionResult.getVelocity(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.velocity.x
                vy = yMotionResult.getVelocity(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.velocity.y
                omega = thetaMotionResult.getVelocity(it.enteredAt?.elapsedNow() ?: return@action false) - odometry.velocity.radians
                odometry.position == targetPosition
            }
        }

        exit {
            it.reset()
            q = createTuningMatrix(xCost, yCost, thetaCost, vxCost, vyCost, omegaCost)
            r = mk.identity<Double>(4) * controlCost
            lqr = LQR(jacobians.a, jacobians.b, q, r)
        }
    }

    fun moveToPositionNoProfile(targetPosition: Pose) : Command<DrivetrainState> = Command.create("Move To Position", state) {
        action {
            with(it) {
                x = targetPosition.x - odometry.position.x
                y = targetPosition.y - odometry.position.y
                theta = targetPosition.radians - odometry.position.radians
                vx = 0.0 - odometry.velocity.x
                vy = 0.0 - odometry.velocity.y
                omega = 0.0 - odometry.velocity.radians
                odometry.position == targetPosition
            }
        }

        exit { it.reset() }
    }

    fun setXControl(x: Double): Command<DrivetrainData> = Command.instant("Set X Control", data) { it.xControl = -x }

    fun setYControl(y: Double): Command<DrivetrainData> = Command.instant("Set Y Control", data) { it.yControl = -y }

    private var rotMulti = 1 - (data.xControl.absoluteValue.coerceAtLeast(data.yControl.absoluteValue) * .6)
    fun setRotControl(rot: Double): Command<DrivetrainData> = Command.instant("Set Rot Control", data) { it.rotControl = -rot * rotMulti }

    override fun bindControls(
        profile: Profile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.drivetrain) {
            if (!gamepad.matches(desiredGamepad)) return
            data.driveOrientation = orientation

            builder.register(x) { setXControl(it * x.modifier) }
            builder.register(y) { setYControl(it * y.modifier) }
            builder.register(rot) { setRotControl(it * rot.modifier) }
        }

    data class DrivetrainData(
        /* We only need to set these if we are using driver control */
        var driveOrientation: DriveOrientation? = null,
        var xControl: Double = 0.0,
        var yControl: Double = 0.0,
        var rotControl: Double = 0.0,
    ) : BaseCommandState() { val isInTeleop get() = driveOrientation != null }

    data class DrivetrainState(
        var x: Double = 0.0,
        var y: Double = 0.0,
        var theta: Double = 0.0,
        var vx: Double = 0.0,
        var vy: Double = 0.0,
        var omega: Double = 0.0,

        var ufl: Double = 0.0,
        var ufr: Double = 0.0,
        var ubl: Double = 0.0,
        var ubr: Double = 0.0,
    ) : BaseCommandState() {
        val stateMatrix : D2Array<Double>
            get() = doubleArrayOf(x, y, theta, vx, vy, omega).toMK2D()

        var controlMatrix: D2Array<Double> = doubleArrayOf(ufl, ufr, ubl, ubr).toMK2D()
            set(matrix) {
                with(matrix.data) {
                    ufl = this[0]
                    ufr = this[1]
                    ubl = this[2]
                    ubr = this[3]
                }
                field = matrix
            }
            get() = doubleArrayOf(ufl, ufr, ubl, ubr).toMK2D()

        fun reset() {
            x = 0.0
            y = 0.0
            theta = 0.0
            vx = 0.0
            vy = 0.0
            omega = 0.0
            ufl = 0.0
            ufr = 0.0
            ubl = 0.0
            ubr = 0.0
        }
    }

    inner class JacobianCalculations {
        private val cA = (1 / sqrt(2.0) * STALL_TORQUE) / (WHEEL_RADIUS * ROBOT_MASS * WHEEL_RADIUS * FREE_SPEED)

        val a: () -> D2Array<Double> = {
            with(state) {
                arrayOf(
                    doubleArrayOf(0.0, 0.0, -vx * sin(theta) - vy * cos(theta), cos(theta), -sin(theta), 0.0),
                    doubleArrayOf(0.0, 0.0, vx * cos(theta) - vy * sin(theta), sin(theta), cos(theta), 0.0),
                    doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
                    doubleArrayOf(0.0, 0.0, 0.0,
                        cA * (
                            - ufl * sign(vx - vy - omega * (LX + LY))
                            + ufr * sign(vx + vy + omega * (LX + LY))
                            + ubl * sign(vx + vy - omega * (LX + LY))
                            - ubr * sign(vx - vy + omega * (LX + LY))),
                        cA * (
                            + ufl * sign(vx - vy - omega * (LX + LY))
                            + ufr * sign(vx + vy + omega * (LX + LY))
                            + ubl * sign(vx + vy - omega * (LX + LY))
                            + ubr * sign(vx - vy + omega * (LX + LY))),
                        (cA * (LX + LY)) * (
                            + ufl * sign(vx - vy - omega * (LX + LY))
                            + ufr * sign(vx + vy + omega * (LX + LY))
                            - ubl * sign(vx + vy - omega * (LX + LY))
                            - ubr * sign(vx - vy + omega * (LX + LY)))
                    ),
                    doubleArrayOf(0.0, 0.0, 0.0,
                        cA * (
                            - ufl * sign(vx - vy - omega * (LX + LY))
                            - ufr * sign(vx + vy + omega * (LX + LY))
                            - ubl * sign(vx + vy - omega * (LX + LY))
                            - ubr * sign(vx - vy + omega * (LX + LY))),
                        cA * (
                            + ufl * sign(vx - vy - omega * (LX + LY))
                            - ufr * sign(vx + vy + omega * (LX + LY))
                            - ubl * sign(vx + vy - omega * (LX + LY))
                            + ubr * sign(vx - vy + omega * (LX + LY))),
                        (cA * (LX + LY)) * (
                            + ufl * sign(vx - vy - omega * (LX + LY))
                            - ufr * sign(vx + vy + omega * (LX + LY))
                            + ubl * sign(vx + vy - omega * (LX + LY))
                            - ubr * sign(vx - vy + omega * (LX + LY)))
                    ),
                    doubleArrayOf(0.0, 0.0, 0.0,
                        (cA / (LX + LY)) * (
                            + ufl * sign(vx - vy - omega * (LX + LY))
                            - ufr * sign(vx + vy + omega * (LX + LY))
                            + ubl * sign(vx + vy - omega * (LX + LY))
                            - ubr * sign(vx - vy + omega * (LX + LY))),
                        (cA / (LX + LY)) * (
                            - ufl * sign(vx - vy - omega * (LX + LY))
                            - ufr * sign(vx + vy + omega * (LX + LY))
                            + ubl * sign(vx + vy - omega * (LX + LY))
                            + ubr * sign(vx - vy + omega * (LX + LY))),
                        cA * (
                            + ufl * sign(vx - vy - omega * (LX + LY))
                            - ufr * sign(vx + vy + omega * (LX + LY))
                            + ubl * sign(vx + vy - omega * (LX + LY))
                            - ubr * sign(vx - vy + omega * (LX + LY)))
                    )
                ).toMK2D()
            }
        }

        private val cB = (1 / sqrt(2.0) * STALL_TORQUE) / (WHEEL_RADIUS * ROBOT_MASS)

        val b: () -> D2Array<Double> = {
            with(state) {
                arrayOf(
                    doubleArrayOf(0.0, 0.0, 0.0, 0.0),
                    doubleArrayOf(0.0, 0.0, 0.0, 0.0),
                    doubleArrayOf(0.0, 0.0, 0.0, 0.0),
                    doubleArrayOf(
                        + cB * (1 - (abs(vx - vy - omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        - cB * (1 - (abs(vx + vy + omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        - cB * (1 - (abs(vx + vy - omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        + cB * (1 - (abs(vx - vy + omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED)))
                    ),
                    doubleArrayOf(
                        + cB * (1 - (abs(vx - vy - omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        + cB * (1 - (abs(vx + vy + omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        + cB * (1 - (abs(vx + vy - omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        + cB * (1 - (abs(vx - vy + omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED)))
                    ),
                    doubleArrayOf(
                        - (cB / (LX + LY)) * (1 - (abs(vx - vy - omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        + (cB / (LX + LY)) * (1 - (abs(vx + vy + omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        - (cB / (LX + LY)) * (1 - (abs(vx + vy - omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED))),
                        + (cB / (LX + LY)) * (1 - (abs(vx - vy + omega * (LX + LY)) / (WHEEL_RADIUS * FREE_SPEED)))
                    )
                ).toMK2D()
            }
        }
    }
}

private data class DriveMotors(
    val frontLeft: DcMotorEx,
    val backLeft: DcMotorEx,
    val frontRight: DcMotorEx,
    val backRight: DcMotorEx
) : Iterable<DcMotorEx> {
    constructor(
        frontLeft: String,
        backLeft: String,
        frontRight: String,
        backRight: String,
        hardwareMap: HardwareMap
    ) : this(
        frontLeft = hardwareMap.getByName(frontLeft),
        backLeft = hardwareMap.getByName(backLeft),
        frontRight = hardwareMap.getByName(frontRight),
        backRight = hardwareMap.getByName(backRight)
    )

    fun setPower(matrix: D2Array<Double>) {
        setPower(matrix.data[0], matrix.data[1], matrix.data[2], matrix.data[3])
    }

    fun setPower(frontLeft: Double, backLeft: Double, frontRight: Double, backRight: Double) {
        val ratio = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight), 1.0)

        this.frontLeft.power = frontLeft / ratio
        this.backLeft.power = backLeft / ratio
        this.frontRight.power = frontRight / ratio
        this.backRight.power = backRight / ratio

        WebData.setDrivetrain(frontLeft, -frontRight, backLeft, -backRight)
    }

    fun setPower(power: Double) {
        setPower(power, power, power, power)
    }

    override operator fun iterator() = listOf(frontLeft, backLeft, frontRight, backRight).iterator()
}