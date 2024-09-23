package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.axiom.commands.*
import org.firstinspires.ftc.teamcode.axiom.input.GamepadSystem
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

inline fun <reified T : HardwareDevice> HardwareMap.getByName(name: String): T {
    return this.get(T::class.java, name)
}

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

    companion object {
        fun default(motors: Motors): DrivetrainState {
            return object : DrivetrainState, CommandState by CommandState.default("Drivetrain") {
                override val motors = motors
                override var targetPose = Pose(0.0, 0.0, 0.0)
                override var mode = Drivetrain.ControlMode.DRIVER_CONTROL
            }
        }
    }
}


class Drivetrain(hardwareMap: HardwareMap, gamepadSystem: GamepadSystem, private val odometrySystem: OdometrySystem) : System {
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
            Mecanum.fieldDriverControl(it, odometrySystem, referencePose)
//            Mecanum.robotDriverControl(it)

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
            val rotation = -gamepad1.rightJoystick.value.x
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

            val heading = odometry.pose.radians - referencePose.radians

            val angleX =
                gamepad1.leftJoystick.value.x * cos(-heading) - gamepad1.leftJoystick.value.y * sin(
                    -heading
                )
            val angleY =
                gamepad1.leftJoystick.value.x * sin(-heading) + gamepad1.leftJoystick.value.y * cos(
                    -heading
                )

            val rotation = gamepad1.rightJoystick.value.x

            val powers = calculatePowers(angleX, angleY, rotation)

            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
        }
//
//        fun moveToPosition(state: DrivetrainState, targetPose: Pose) {
//            val pose = state.targetPose
//            val position = pose.position
//            val rotation = pose.\
//
//            val targetPosition = targetPose.position
//            val targetRotation = targetPose.rotation
//
//            val positionError = targetPosition - position
//            val rotationError = targetRotation - rotation
//
//            val x = positionError.x
//            val y = positionError.y
//            val rotation = rotationError
//
//            val powers = calculatePowers(x, y, rotation)
//            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
//        }
    }

    fun setMode(mode: ControlMode) {
        beforeRun.state.mode = mode
    }

    fun setTargetPose(pose: Pose) {
        beforeRun.state.targetPose = pose
    }

    fun moveToPose(pose: Pose) {
        setTargetPose(pose)
        setMode(ControlMode.AUTONOMOUS)
    }
}