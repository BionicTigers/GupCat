package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.axiom.commands.*
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Time
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

data class Motors(
    val frontLeft: DcMotorEx,
    val backLeft: DcMotorEx,
    val frontRight: DcMotorEx,
    val backRight: DcMotorEx
) {
    fun setPower(frontLeft: Double, backLeft: Double, frontRight: Double, backRight: Double) {
        this.frontLeft.power = frontLeft
        this.backLeft.power = backLeft
        this.frontRight.power = frontRight
        this.backRight.power = backRight
    }

    fun setPower(power: Double) {
        setPower(power, power, power, power)
    }

    operator fun iterator() = listOf(frontLeft, backLeft, frontRight, backRight).iterator()
}

interface DrivetrainState : CommandState {
    val motors: Motors
    var targetPose: Pose
    var mode: Drivetrain.ControlMode

    val pidX: PID
    val pidY: PID
    val pidRot: PID

    var profileX: MotionResult?
    var profileY: MotionResult?
    var profileRot: MotionResult?

    var timeStarted: Time

    companion object {
        fun default(motors: Motors): DrivetrainState {
            return object : DrivetrainState, CommandState by CommandState.default("Drivetrain") {
                override val motors = motors
                override var targetPose = Pose(0.0, 0.0, 0.0)
                override var mode = Drivetrain.ControlMode.DRIVER_CONTROL

                override val pidX = PID(PIDTerms(1.0, 0.0), 0.0, 0.0, -1.0, 1.0)
                override val pidY = PID(PIDTerms(1.0, 0.0), 0.0, 0.0, -1.0, 1.0)
                override val pidRot = PID(PIDTerms(1.0, 0.0), 0.0, 0.0, -1.0, 1.0)

                override var profileX: MotionResult? = null
                override var profileY: MotionResult? = null
                override var profileRot: MotionResult? = null

                override var timeStarted = Time()
            }
        }
    }
}

class Drivetrain(
    hardwareMap: HardwareMap,
    gamepadSystem: GamepadSystem,
    odometrySystem: OdometrySystem,
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

    override val dependencies: List<System> = listOf(odometrySystem, gamepadSystem)

    override val beforeRun = Command(DrivetrainState.default(motors))
        .setAction {
            if (it.mode == ControlMode.DRIVER_CONTROL)
                Mecanum.fieldDriverControl(it, odometrySystem, referencePose)
            else
                moveToPose(it.targetPose)

            true
        }

    override val afterRun: Command<*>? = null

    init {
        motors.iterator().forEach { motor ->
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private var referencePose = odometrySystem.pose

    object Mecanum {
        fun calculatePowers(x: Double, y: Double, rotation: Double): List<Double> {
            val frontLeft = y - x + rotation
            val frontRight = y + x - rotation
            val backLeft = y + x + rotation
            val backRight = y - x - rotation

            val powers = listOf(-frontLeft, -backLeft, frontRight, backRight)

            val maxPower = powers.maxOrNull()?.let { abs(it) } ?: 1.0
            return powers.map { it / maxPower }
        }

        @Suppress("unused")
        fun robotDriverControl(state: DrivetrainState) {
            val gamepadSystem = GamepadSystem.activeSystem
            if (gamepadSystem == null) {
                state.motors.setPower(0.0)
                return
            }

            val (gamepad1, _) = gamepadSystem.gamepads

            val (x, y) = gamepad1.leftJoystick.value
            val rotation = gamepad1.rightJoystick.value.x
            val powers = calculatePowers(x, y, rotation)
            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
        }

        fun fieldCalculatePowers(x: Double, y: Double, rotation: Double, heading: Double): List<Double> {
            val angleX = x * cos(-heading) - y * sin(-heading)
            val angleY = x * sin(-heading) + y * cos(-heading)

            return calculatePowers(angleX, angleY, rotation)
        }

        fun fieldDriverControl(state: DrivetrainState, odometry: OdometrySystem, referencePose: Pose) {
            val gamepadSystem = GamepadSystem.activeSystem
            if (gamepadSystem == null) {
                state.motors.setPower(0.0)
                return
            }

            val (gamepad1, _) = gamepadSystem.gamepads

            val heading = odometry.pose.radians - referencePose.radians

            val (x, y) = gamepad1.leftJoystick.value
            val rotation = gamepad1.rightJoystick.value.x

            val powers = fieldCalculatePowers(x, y, rotation, heading)
            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
        }

        fun moveToPosition(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
            if (targetPose != null) {
                state.targetPose = targetPose
                state.timeStarted = state.timeInScheduler
            }

            state.profileX = generateMotionProfile(
                odometry.pose.x,
                state.targetPose.x,
                20.0,
                20.0,
                20.0
            )

            state.profileY = generateMotionProfile(
                odometry.pose.y,
                state.targetPose.y,
                20.0,
                20.0,
                20.0
            )

            state.profileRot = generateMotionProfile(
                odometry.pose.radians,
                state.targetPose.radians,
                20.0,
                20.0,
                20.0
            )

            state.pidX.reset()
            state.pidY.reset()
            state.pidRot.reset()

            val pose = odometry.pose

            val powerX = state.pidX.calculate(pose.x, state.profileX!!.getPosition(state.timeInScheduler - state.timeStarted))
            val powerY = state.pidY.calculate(pose.y, state.profileY!!.getPosition(state.timeInScheduler - state.timeStarted))
            val powerRot = state.pidRot.calculate(pose.radians, state.profileRot!!.getPosition(state.timeInScheduler - state.timeStarted))

            val powers = fieldCalculatePowers(powerX, powerY, powerRot, pose.radians)
            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
        }
    }

    fun setMode(mode: ControlMode) {
        beforeRun.state.mode = mode
    }

    var targetPose: Pose
        get() = beforeRun.state.targetPose
        private set(value) {
            beforeRun.state.targetPose = value
        }

    fun moveToPose(pose: Pose) {
        targetPose = pose
        setMode(ControlMode.AUTONOMOUS)
    }
}