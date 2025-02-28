package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import io.github.bionictigers.axiom.commands.*
import org.firstinspires.ftc.teamcode.input.Gamepad
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import io.github.bionictigers.axiom.utils.Time
import io.github.bionictigers.axiom.web.Editable
import io.github.bionictigers.axiom.web.Hidden
import io.github.bionictigers.axiom.web.WebData
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.interpolatedMapOf
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

        WebData.setDrivetrain(frontLeft, -frontRight, backLeft, -backRight)
    }

    fun setPower(power: Double) {
        setPower(power, power, power, power)
    }

    operator fun iterator() = listOf(frontLeft, backLeft, frontRight, backRight).iterator()
}

interface DrivetrainState : CommandState {
    @Hidden
    val motors: Motors
    @Hidden
    val controlHub: ControlHub
    @Hidden
    var beforeMovePose: Pose
    var targetPose: Pose
    @Hidden
    var mode: Drivetrain.ControlMode

    @Hidden
    val pidX: PID
    @Hidden
    val pidY: PID
    @Hidden
    val pidRot: PID

    @Editable
    val pPidX: PID
    @Editable
    val pPidY: PID
    @Editable
    val pPidRot: PID

    @Editable
    val vPidX: PID
    @Editable
    val vPidY: PID
    @Editable
    val vPidRot: PID

    var profileX: MotionResult?
    var profileY: MotionResult?

    var profileDist: MotionResult?
    var profileRot: MotionResult?

    @Hidden
    var moveToPositionTimeout: Time?

    @Hidden
    var timeStarted: Time

    var posXSp: Double
    var posYSp: Double
    var posRSp: Double

    var velRSp: Double

    @Editable
    @Hidden
    val xKFV: Double
    @Editable
    @Hidden
    val yKFV: Double
    @Editable
    @Hidden
    val rKFV: Double
    @Editable
    @Hidden
    val sKP: Double

    companion object {
        fun default(motors: Motors, hardwareMap: HardwareMap): DrivetrainState {
            return object : DrivetrainState, CommandState by CommandState.default("Drivetrain") {
                override val controlHub: ControlHub = ControlHub(hardwareMap, "Control Hub")

                override val xKFV = 95.6412
                override val yKFV = 123.7239
                override val rKFV = 123.7239
                override val sKP = .115

                override val motors = motors
                override var beforeMovePose: Pose = Pose(0.0,0.0,0.0)
                override var targetPose = Pose(0.0, 0.0, 0.0)
                override var mode = Drivetrain.ControlMode.DRIVER_CONTROL

                override val pPidX = PID(PIDTerms(5000.0, 100.0), 0.0, 3657.6, -1104.0, 1104.0, 100)
                override val pPidY = PID(PIDTerms(5000.0, 100.0), 0.0, 3657.6, -1427.0, 1427.0, 100)
                override val pPidRot = PID(PIDTerms(80.0, 40.0), -2 * PI, 2 * PI, Angle.degrees(-294.81).radians, Angle.degrees(294.81).radians, 100)

                override val vPidX = PID(PIDTerms(3.5, 3.5), -1104.0, 1104.0, -1.0, 1.0, 40)
                override val vPidY = PID(PIDTerms(3.5,  3.5), -1427.0, 1427.0, -1.0, 1.0, 40)
                override val vPidRot = PID(PIDTerms(2.0, 10.0), Angle.degrees(-294.81).radians, Angle.degrees(294.81).radians, -1.0, 1.0)

                override val pidX = PID(PIDTerms(22.0, 20.0), 0.0, 3657.6, -1.0, 1.0)
                override val pidY = PID(PIDTerms(22.0, 20.0), 0.0, 3657.6, -1.0, 1.0)
                override val pidRot = PID(PIDTerms(16.0, 60.0), -2 * PI, 2 * PI, -1.0, 1.0)

                override var posXSp = 0.0
                override var posYSp = 0.0
                override var posRSp = 0.0

                override var velRSp = 0.0

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
    var disabled = false

    override val beforeRun = Command(DrivetrainState.default(motors, hardwareMap))
        .setOnEnter {
//            imu.initialize(BNO055IMUNew.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))
            referencePose = odometrySystem.globalPose
        }
        .setAction {
            if (disabled) return@setAction false

            if (it.mode == ControlMode.DRIVER_CONTROL) {
                Mecanum.fieldDriverControl(it, odometrySystem, referencePose)
//                Mecanum.robotDriverControl(it)
            } else {
                if (moveFinished) {
                    stop()
                }

                Mecanum.newMoveToPosition(it, odometrySystem)
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

    fun veloTest(power: Double, direction: Pose) = Mecanum.velocityCoeffTest(beforeRun.state, power, direction)

    object Mecanum {
        // TODO: update these values and figure out how to actually measure jerk instead of making it up
        val xJerk = 4000.0
        val yJerk = 4000.0
        val angularJerk = Angle.degrees(400.0)
        val xMaxAcceleration = 7441.0
        val yMaxAcceleration = 8087.0
        val angularMaxAcceleration = Angle.degrees(130.7315) //130
        val xMaxVelocity = 1104.0
        val yMaxVelocity = 1427.0
        val angularMaxVelocity = Angle.degrees(294.81)

        val yvPIDPMap = interpolatedMapOf(
            0.0 to 3.0,
            200.0 to 3.0,
            800.0 to 7.0,
            1427.0 to 9.0,
        )
        val ypPIDPMap = interpolatedMapOf(
            100.0 to 5000.0,
            1000.0 to 6000.0,
        )

        val xvPIDPMap = interpolatedMapOf(
            0.0 to 3.0,
            200.0 to 3.0,
            800.0 to 7.0,
            1104.0 to 9.0,
        )

        val rvPIDPMap = interpolatedMapOf(
            0.0 to 2.5,
            Angle.degrees(30.0).radians to 3.0,
            Angle.degrees(294.81).radians to 6.0,
        )

        fun calculatePowers(x: Double, y: Double, rotation: Double): List<Double> {
            val frontLeft = y - x + rotation
            val frontRight = y + x - rotation
            val backLeft = y + x + rotation
            val backRight = y - x - rotation

            val powers = listOf(-frontLeft, -backLeft, frontRight, backRight)

            return powers.map { it }
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

/*        fun moveToPosition(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
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

            val powerX = state.vPidX.calculate(pose.x, state.profileX!!.getPosition(state.timeInScheduler - state.timeStarted))
            val powerY = state.vPidY.calculate(pose.y, state.profileY!!.getPosition(state.timeInScheduler - state.timeStarted))
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
            val powers = fieldCalculatePowers(-powerX, powerY, powerRot, pose.radians)
            val modifier = 1
//            powers = fieldCalculatePowers(1.0, 0.0, 0.0, pose.radians)
            state.motors.setPower(powers[0] * modifier, powers[1] * modifier, powers[2] * modifier, powers[3] * modifier)
        }*/

        var beta: Double = 0.0
        var starting: Vector2 = Vector2()
        var rotationFinished = false

        fun ffMoveToPosition(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
            val pose = odometry.globalPose
            val (velocity, angVelocity) = odometry.globalVelocity

            if (targetPose != null) {
                beta = atan2((state.targetPose.x - pose.x), (state.targetPose.y - pose.y))
                starting = odometry.globalPose.position

                state.targetPose = targetPose
                state.timeStarted = state.timeInScheduler

                state.pPidX.reset()
                state.pPidY.reset()
                state.pPidRot.reset()
                state.vPidX.reset()
                state.vPidY.reset()
                state.vPidRot.reset()

                state.profileRot = generateMotionProfile(
                    odometry.globalPose.radians,
                    state.targetPose.radians,
                    angularJerk.radians,
                    angularMaxAcceleration.radians,
                    angularMaxVelocity.radians,
                )

                // TODO: Iterate trueMax over all rotations throughout the move to find the lowest and apply it instead of just the starting max
                state.profileDist = generateMotionProfile(
                    0.0,
                    (state.targetPose.position - odometry.globalPose.position).magnitude(),
                    yJerk,
                    yMaxAcceleration,
                    yMaxVelocity
                )
            }

            val xPosition =
                starting.x + sin(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)
            val yPosition =
                starting.y + cos(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)

            val yVelocity = cos(beta) * state.profileDist!!.getVelocity(state.timeInScheduler - state.timeStarted)

            val yPower = yVelocity / state.yKFV / state.controlHub.getVoltage()
            println(yPower)
            val powers = fieldCalculatePowers(0.0, -yPower, -0.0, odometry.globalPose.radians)

            val modifier = 1
            state.motors.setPower(powers[0] * modifier, powers[1] * modifier, powers[2] * modifier, powers[3] * modifier)
        }

        fun newMoveToPosition(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
            val pose = odometry.globalPose
            val (vel, ang) = odometry.globalVelocity

            if (targetPose != null) {
                beta = atan2((state.targetPose.x - pose.x), (state.targetPose.y - pose.y))
                starting = odometry.globalPose.position
                rotationFinished = false

                state.targetPose = targetPose
                state.timeStarted = state.timeInScheduler

                state.pPidX.reset()
                state.pPidY.reset()
                state.pPidRot.reset()
                state.vPidX.reset()
                state.vPidY.reset()
                state.vPidRot.reset()

                state.profileRot = generateMotionProfile(
                    odometry.globalPose.radians,
                    state.targetPose.radians,
                    angularJerk.radians,
                    angularMaxAcceleration.radians,
                    angularMaxVelocity.radians,
                )

                // TODO: Iterate trueMax over all rotations throughout the move to find the lowest and apply it instead of just the starting max
                state.profileDist = generateMotionProfile(
                    0.0,
                    (state.targetPose.position - odometry.globalPose.position).magnitude(),
                    yJerk,
                    yMaxAcceleration,
                    yMaxVelocity
                )
            }

            val xPosition = starting.x + sin(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)
            val yPosition = starting.y + cos(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)
            val rotPosition =
                if (!rotationFinished) state.profileRot!!.getPosition(state.timeInScheduler - state.timeStarted)
                else state.targetPose.radians

            state.posXSp = xPosition
            state.posYSp = yPosition
            state.posRSp = Math.toDegrees(rotPosition)

            // position PIDs, output is velocity
            val xVelocity = state.pPidX.calculate(xPosition, pose.x)
            state.pPidY.p = ypPIDPMap[yPosition]
            val yVelocity = state.pPidY.calculate(yPosition, pose.y)
            val rotVelocity = state.pPidRot.calculate(rotPosition, pose.radians)

            state.velRSp = Math.toDegrees(rotVelocity)


            state.vPidX.p = xvPIDPMap[xVelocity]
            var xPower = state.vPidX.calculate(xVelocity, vel.x)

            state.vPidY.p = yvPIDPMap[yVelocity]
            var yPower = state.vPidY.calculate(yVelocity, vel.y)

            var rotPower = state.vPidRot.calculate(rotVelocity, ang.radians)

            if (!rotationFinished) {
                println("${pose.rotation.degrees} ${state.targetPose.rotation.degrees} ${pose.rotation - state.targetPose.rotation} ${(pose.rotation - state.targetPose.rotation).abs.degrees}")
                if ((pose.rotation - state.targetPose.rotation).abs.degrees > 5) {
                    xPower = 0.0
                    yPower = 0.0
                } else {
                    yPower = 0.0
                    xPower = 0.0
                    state.pPidX.reset()
                    state.pPidY.reset()
                    state.vPidX.reset()
                    state.vPidY.reset()
                    beta = atan2((state.targetPose.x - pose.x), (state.targetPose.y - pose.y))
                    state.timeStarted = state.timeInScheduler
                    starting = pose.position
                    state.profileDist = generateMotionProfile(
                        0.0,
                        (state.targetPose.position - pose.position).magnitude(),
                        yJerk,
                        yMaxAcceleration,
                        yMaxVelocity
                    )
                    rotationFinished = true
                }
            }

            val powers = fieldCalculatePowers(xPower, -yPower, -rotPower, odometry.globalPose.radians)

            val modifier = 1
            state.motors.setPower(powers[0] * modifier, powers[1] * modifier, powers[2] * modifier, powers[3] * modifier)
        }

        fun velocityCoeffTest(state: DrivetrainState, power: Double, direction: Pose) {
            val x = power * direction.x
            val y = power * direction.y
            val r = power * direction.radians

            val powers = fieldCalculatePowers(x, -y, -r, 0.0)
            state.motors.setPower(powers[0], powers[1], powers[2], powers[3])
        }

        // TODO: Use kinematics to actually make sure this function is correct (its not)
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
            val compare = Pose(5, 5, 5)
            val diff = beforeRun.state.timeInScheduler - beforeRun.state.timeStarted
//            println("Within: ${odometrySystem.globalPose.within(targetPose, compare)}, Timeout: ${(beforeRun.state.moveToPositionTimeout != null && diff > beforeRun.state.moveToPositionTimeout!!)}")
            return odometrySystem.globalPose.within(targetPose, compare)// && (beforeRun.state.moveToPositionTimeout != null && diff > beforeRun.state.moveToPositionTimeout!!)
        }

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.getBooleanButton(Gamepad.Buttons.B).onDown {
            reset()
        }
    }

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
        Mecanum.newMoveToPosition(beforeRun.state, odometrySystem, pose)
        beforeRun.state.moveToPositionTimeout = timeout
        setMode(ControlMode.AUTONOMOUS)
    }

    fun logMotorPowers(telemetry: Telemetry) {
//        for (motor in motors)
//            telemetry.addData(motor.deviceName, motor.power)
    }
}