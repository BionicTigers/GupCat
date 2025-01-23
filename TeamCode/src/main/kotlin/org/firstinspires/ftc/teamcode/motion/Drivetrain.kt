package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.axiom.commands.*
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Time
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

data class Motors(
    val frontLeft: DcMotorEx,
    val backLeft: DcMotorEx,
    val frontRight: DcMotorEx,
    val backRight: DcMotorEx
) {
    fun setPower(frontLeft: Double, backLeft: Double, frontRight: Double, backRight: Double) {
        val ratio = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight), 1.0)

        this.frontLeft.power = frontLeft / ratio
        this.backLeft.power = backLeft / ratio
        this.frontRight.power = frontRight / ratio
        this.backRight.power = backRight / ratio
    }

    fun setPower(power: Double) {
        setPower(power, power, power, power)
    }

    operator fun iterator() = listOf(frontLeft, backLeft, frontRight, backRight).iterator()
}

interface DrivetrainState : CommandState {
    val motors: Motors
    var beforeMovePose: Pose
    var targetPose: Pose
    var mode: Drivetrain.ControlMode

    val pPidX: PID
    val pPidY: PID
    val pPidRot: PID

    val vPidX: PID
    val vPidY: PID
    val vPidRot: PID

    var profileX: MotionResult?
    var profileY: MotionResult?

    var profileDist: MotionResult?
    var profileRot: MotionResult?

    var moveToPositionTimeout: Time?

    var timeStarted: Time

    companion object {
        fun default(motors: Motors): DrivetrainState {
            return object : DrivetrainState, CommandState by CommandState.default("Drivetrain") {
                override val motors = motors
                override var beforeMovePose: Pose = Pose(0.0,0.0,0.0)
                override var targetPose = Pose(0.0, 0.0, 0.0)
                override var mode = Drivetrain.ControlMode.DRIVER_CONTROL

                override val pPidX = PID(PIDTerms(30.0, 40.0), 0.0, 3657.6, 0.0, 3657.6)
                override val pPidY = PID(PIDTerms(30.0, 40.0), 0.0, 3657.6, 0.0, 3657.6)
                override val pPidRot = PID(PIDTerms(18.0, 90.0), -2 * PI, 2 * PI, -2 * PI, 2 * PI)

                // TODO: tune velocity pid values these ones are made up
                override val vPidX = PID(PIDTerms(30.0, 40.0), 0.0, 3657.6, -1.0, 1.0)
                override val vPidY = PID(PIDTerms(30.0, 40.0), 0.0, 3657.6, -1.0, 1.0)
                override val vPidRot = PID(PIDTerms(18.0, 90.0), -2 * PI, 2 * PI, -1.0, 1.0)

                override var profileX: MotionResult? = null
                override var profileY: MotionResult? = null
                override var profileDist: MotionResult? = null
                override var profileRot: MotionResult? = null
                override var moveToPositionTimeout: Time? = null

                override var timeStarted = Time()
            }
        }
    }
}

class Drivetrain(
    hardwareMap: HardwareMap,
    gamepadSystem: GamepadSystem,
    private val odometrySystem: OdometrySystem,
) : System {
    enum class ControlMode {
        AUTONOMOUS,
        DRIVER_CONTROL
    }

    private val motors = Motors(
        hardwareMap.getByName("frontLeft"),
        hardwareMap.getByName("backLeft"),
        hardwareMap.getByName("frontRight"),
        hardwareMap.getByName("backRight")
    )
//    val imu = hardwareMap.getByName<BNO055IMUNew>("InternalIMU")

    override val dependencies: List<System> = listOf(odometrySystem, gamepadSystem)

    override val beforeRun = Command(DrivetrainState.default(motors))
        .setOnEnter {
//            imu.initialize(BNO055IMUNew.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))
            referencePose = odometrySystem.globalPose
        }
        .setAction {
            if (it.mode == ControlMode.DRIVER_CONTROL) {
                Mecanum.fieldDriverControl(it, odometrySystem, referencePose)
            } else {
                if (moveFinished) {
                    stop()
                }

                Mecanum.moveToPosition(it, odometrySystem)
            }

            false
        }

    override val afterRun: Command<*>? = null

    init {
        motors.iterator().forEach { motor ->
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private var referencePose = odometrySystem.globalPose

    object Mecanum {
        // TODO: update these values and figure out how to actually measure jerk instead of making it up
        val xJerk = 2000.0
        val yJerk = 2000.0
        val angularJerk = 2000.0
        val xMaxAcceleration = 7441.0
        val yMaxAcceleration = 8087.0
        val angularMaxAcceleration = Angle.degrees(81.7315).radians
        val xMaxVelocity = 1104.0
        val yMaxVelocity = 1427.0
        val angularMaxVelocity = Angle.degrees(294.81)

        fun calculatePowers(x: Double, y: Double, rotation: Double): List<Double> {
            val frontLeft = y - x + rotation
            val frontRight = y + x - rotation
            val backLeft = y + x + rotation
            val backRight = y - x - rotation

            val powers = listOf(-frontLeft, -backLeft, frontRight, backRight)

//            val maxPower = powers.maxOrNull()?.let { abs(it) } ?: 1.0
            return powers.map { it /*/ maxPower*/ }
        }

        @Suppress("unused")
        fun robotDriverControl(state: DrivetrainState) {
            val gamepadSystem = GamepadSystem.activeSystem
            if (gamepadSystem == null) {
                state.motors.setPower(0.0)
                return
            }

            val (gamepad1, _) = gamepadSystem.gamepads

            val (x, y) = (gamepad1.leftJoystick.value)
            val rotation = -gamepad1.rightJoystick.value.x

//            val (x, y) = gamepad1.rightJoystick.value
//            val rotation = gamepad1.leftTrigger.value - gamepad1.rightTrigger.value

            val powers = calculatePowers(x, y, rotation)
            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
        }

        fun fieldDriverControl(state: DrivetrainState, odometry: OdometrySystem, referencePose: Pose) {
            val gamepadSystem = GamepadSystem.activeSystem
            if (gamepadSystem == null) {
                state.motors.setPower(0.0)
                return
            }

            val (gamepad1, _) = gamepadSystem.gamepads

            val heading = odometry.globalPose.radians - referencePose.radians

            val (x, y) = gamepad1.leftJoystick.value
            val rotation = -gamepad1.rightJoystick.value.x
//            val (x, y) = gamepad1.rightJoystick.value
//            val rotation = gamepad1.leftTrigger.value - gamepad1.rightTrigger.value

            val powers = fieldCalculatePowers(x, y, rotation, heading)
            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
        }

        fun fieldCalculatePowers(x: Double, y: Double, rotation: Double, heading: Double): List<Double> {
            val angleX = x * cos(-heading) - y * sin(-heading)
            val angleY = x * sin(-heading) + y * cos(-heading)

            return calculatePowers(angleX, angleY, rotation)
        }

        fun moveToPosition(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
            if (targetPose != null) {
                state.targetPose = targetPose
                state.timeStarted = state.timeInScheduler

                state.pPidX.reset()
                state.pPidY.reset()
                state.pPidRot.reset()

                state.profileX = generateMotionProfile(
                    odometry.globalPose.x,
                    state.targetPose.x,
                    xJerk,
                    xMaxAcceleration,
                    xMaxVelocity,
//                    odometry.beforeRun.state.velocity.x
                )
                println()

                state.profileY = generateMotionProfile(
                    odometry.globalPose.y,
                    state.targetPose.y,
                    yJerk,
                    yMaxAcceleration,
                    yMaxVelocity,
//                    odometry.beforeRun.state.velocity.y
                )
            }

            val pose = odometry.globalPose

            val powerX = state.pPidX.calculate(pose.x, state.profileX!!.getPosition(state.timeInScheduler - state.timeStarted))
            val powerY = state.pPidY.calculate(pose.y, state.profileY!!.getPosition(state.timeInScheduler - state.timeStarted))
            val targetx = state.profileX!!.getPosition(state.timeInScheduler - state.timeStarted)
            val targety = state.profileY!!.getPosition(state.timeInScheduler - state.timeStarted)
            println("TargetX: $targetx")
            println("TargetY: $targety")

//            val powerRot = state.pidRot.calculate(pose.radians, state.profileRot!!.getPosition(state.timeInScheduler - state.timeStarted))

//            println("X: ${state.profileX!!.getPosition(state.timeInScheduler - state.timeStarted)}")
//            println("Y: ${state.profileY!!.getPosition(state.timeInScheduler - state.timeStarted)}")

//            val powerX = state.pidX.calculate(pose.x, state.targetPose.x)
//            val powerY = state.pidY.calculate(pose.y, state.targetPose.y)
            val powerRot = state.pPidRot.calculate(pose.radians, state.targetPose.radians)
            println("powerX $powerX")
            println("powerY $powerY")
            println("powerRot $powerRot")
            val powers = fieldCalculatePowers(powerX, powerY, powerRot, pose.radians)
            val modifier = 1
//            powers = fieldCalculatePowers(1.0, 0.0, 0.0, pose.radians)
            state.motors.setPower(powers[0] * modifier, powers[1] * modifier, powers[2] * modifier, powers[3] * modifier)
        }

        fun newMoveToPosition(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
            val pose = odometry.globalPose
//            val velocity = odometry.globalVelocity

            val beta = atan2((state.targetPose.x - pose.x), (state.targetPose.y - pose.y))

            if (targetPose != null) {
                state.targetPose = targetPose
                state.timeStarted = state.timeInScheduler

                state.pPidX.reset()
                state.pPidY.reset()
                state.pPidRot.reset()

                state.profileRot = generateMotionProfile(
                    odometry.globalPose.radians,
                    state.targetPose.radians,
                    angularJerk,
                    angularMaxAcceleration,
                    angularMaxVelocity.radians,
                )

                // TODO: Iterate trueMax over all rotations throughout the move to find the lowest and apply it instead of just the starting max
                state.profileDist = generateMotionProfile(
                    odometry.globalPose.position.magnitude(),
                    state.targetPose.position.magnitude(),
                    yJerk*.5,
                    yMaxAcceleration*.5,
                    yMaxVelocity*.5
                )
            }

            // position PIDs, output is velocity
            val xVelocity = state.pPidX.calculate(pose.x, (sin(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)))
            val yVelocity = state.pPidY.calculate(pose.y, (cos(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)))
            val rotVelocity = state.pPidRot.calculate(pose.radians, state.profileRot!!.getPosition(state.timeInScheduler - state.timeStarted))

            println(sin(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted))
            println(cos(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted))

//            // velocity PIDs, output is power
//            val xPower = state.vPidX.calculate(velocity.first.x, xVelocity)
//            val yPower = state.vPidY.calculate(velocity.first.y, yVelocity)
//            val rotPower = state.vPidRot.calculate(velocity.second.radians, rotVelocity)

            val powers = fieldCalculatePowers(xVelocity, yVelocity, rotVelocity, odometry.globalPose.radians)

            val modifier = 1
            state.motors.setPower(powers[0] * modifier, powers[1] * modifier, powers[2] * modifier, powers[3] * modifier)
        }

        // TODO: Use mecanum kinematics to actually make sure this function is correct (its not)
        private fun trueMax(beta: Double, maxX: Double, maxY: Double, odometry: OdometrySystem): Double {
            val difference = abs(odometry.globalPose.radians - beta)
            val gamma = atan(maxX/maxY)

            return (sin(180.0 - difference - gamma) / maxY) / sin(gamma)
        }
    }

    var targetPose: Pose
        get() = beforeRun.state.targetPose
        private set(value) {
            beforeRun.state.targetPose = value
        }

    val moveFinished: Boolean
        get() {
            val compare = Pose(15.0, 15.0, 7.5)
            val diff = beforeRun.state.timeInScheduler - beforeRun.state.timeStarted
//            println("Within: ${odometrySystem.globalPose.within(targetPose, compare)}, Timeout: ${(beforeRun.state.moveToPositionTimeout != null && diff > beforeRun.state.moveToPositionTimeout!!)}")
            return odometrySystem.globalPose.within(targetPose, compare)// && (beforeRun.state.moveToPositionTimeout != null && diff > beforeRun.state.moveToPositionTimeout!!)
        }

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.getBooleanButton(Gamepad.Buttons.B).onDown {
            reset()
        }
    }

//    fun move() {
//        motors.setPower(1.0,1.0,1.0,1.0)
//    }

    //    var percentFinished: Double
//        get() {
//            val diff = (targetPose - odometrySystem.globalPose).absoluteValue
//            val sum =
//        }

    fun setMode(mode: ControlMode) {
        beforeRun.state.mode = mode
    }

    fun stop() {
        setMode(ControlMode.DRIVER_CONTROL)
        motors.setPower(0.0)
    }

    fun reset() {
        referencePose = odometrySystem.globalPose
    }

    fun moveToPose(x: Number, y: Number, rot: Number, timeout: Time? = null) {
        moveToPose(Pose(x.toDouble(), y.toDouble(), rot.toDouble()), timeout)
    }

    fun moveToPose(pose: Pose, timeout: Time? = null) {
        targetPose = pose
        Mecanum.moveToPosition(beforeRun.state, odometrySystem, pose)
        beforeRun.state.moveToPositionTimeout = timeout
        setMode(ControlMode.AUTONOMOUS)
    }

    fun logMotorPowers(telemetry: Telemetry) {
        telemetry.addData("frontLeft", motors.frontLeft.power)
        telemetry.addData("backLeft", motors.backLeft.power)
        telemetry.addData("frontRight", motors.frontRight.power)
        telemetry.addData("backRight", motors.backRight.power)
    }
}