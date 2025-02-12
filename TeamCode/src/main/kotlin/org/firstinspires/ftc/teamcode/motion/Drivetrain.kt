package org.firstinspires.ftc.teamcode.motion

import com.acmerobotics.dashboard.FtcDashboard
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

        WebData.setDrivetrain(frontLeft,frontRight, backLeft, backRight)
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

    val pidX: PID
    val pidY: PID
    val pidRot: PID

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

    val controlHub: ControlHub

    companion object {
        fun default(motors: Motors, hardwareMap: HardwareMap): DrivetrainState {
            return object : DrivetrainState, CommandState by CommandState.default("Drivetrain") {
                override val controlHub: ControlHub = ControlHub(hardwareMap, "Control Hub")

                override val motors = motors
                override var beforeMovePose: Pose = Pose(0.0,0.0,0.0)
                override var targetPose = Pose(0.0, 0.0, 0.0)
                override var mode = Drivetrain.ControlMode.DRIVER_CONTROL

                override val pPidX = PID(PIDTerms(1100.0, 30.0), 0.0, 3657.6, -1104.0, 1104.0, 100)
                override val pPidY = PID(PIDTerms(1100.0, 30.0), 0.0, 3657.6, -1427.0, 1427.0, 100)
                override val pPidRot = PID(PIDTerms(5.0, 70.0), -2 * PI, 2 * PI, Angle.degrees(-294.81).radians, Angle.degrees(294.81).radians, 100)

                override val vPidX = PID(PIDTerms(6.0, 70.0), -1104.0, 1104.0, -1.0, 1.0, 40)
                override val vPidY = PID(PIDTerms(11.5,  70.0), -1427.0, 1427.0, -1.0, 1.0, 40)
                override val vPidRot = PID(PIDTerms(5.0, 50.0), Angle.degrees(-294.81).radians, Angle.degrees(294.81).radians, -1.0, 1.0)

                override val pidX = PID(PIDTerms(22.0, 20.0), 0.0, 3657.6, -1.0, 1.0)
                override val pidY = PID(PIDTerms(22.0, 20.0), 0.0, 3657.6, -1.0, 1.0)
                override val pidRot = PID(PIDTerms(16.0, 60.0), -2 * PI, 2 * PI, -1.0, 1.0)

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

    override val beforeRun = Command(DrivetrainState.default(motors, hardwareMap))
        .setOnEnter {
//            imu.initialize(BNO055IMUNew.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))
            referencePose = odometrySystem.globalPose
        }
        .setAction {
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

    object Mecanum {
        // TODO: update these values and figure out how to actually measure jerk instead of making it up
        val xJerk = 2000.0
        val yJerk = 2000.0
        val angularJerk = 2000.0
        val xMaxAcceleration = 7441.0
        val yMaxAcceleration = 8087.0
        val angularMaxAcceleration = Angle.degrees(81.7315)
        val xMaxVelocity = 1104.0
        val yMaxVelocity = 1427.0
        val angularMaxVelocity = Angle.degrees(294.81)

        val yvPIDPMap = interpolatedMapOf(
            0.0 to 6.0,
            200.0 to 6.0,
            800.0 to 9.0,
            1427.0 to 11.0,
        )

        val xvPIDPMap = interpolatedMapOf(
            0.0 to 6.0,
            200.0 to 6.0,
            800.0 to 9.0,
            1104.0 to 11.0,
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
        }

        var beta: Double = 0.0
        var starting: Vector2 = Vector2()

        fun newMoveToPosition(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
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

                state.profileRot = generateMotionProfile(
                    odometry.globalPose.radians,
                    state.targetPose.radians,
                    angularJerk,
                    angularMaxAcceleration.radians,
                    angularMaxVelocity.radians,
                )

                // TODO: Iterate trueMax over all rotations throughout the move to find the lowest and apply it instead of just the starting max
                state.profileDist = generateMotionProfile(
                    0.0,
                    (state.targetPose.position - odometry.globalPose.position).magnitude(),
                    yJerk*.65,
                    yMaxAcceleration*.65,
                    yMaxVelocity*.65
                )
            }

            val xPosition = starting.x + sin(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)
            val yPosition = starting.y + cos(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)
            val rotPosition = state.profileRot!!.getPosition(state.timeInScheduler - state.timeStarted)

            // position PIDs, output is velocity
//            val xVelocity = state.pPidX.calculate(xPosition, pose.x)
//            val yVelocity = state.pPidY.calculate(yPosition, pose.y)
//            val rotVelocity = state.pPidRot.calculate(rotPosition, pose.radians)

//            println("xVel: $xVelocity")
//            println("yVel: $yVelocity")
//            println("rotVel: $rotVelocity")

//            val xPower = state.vPidX.calculate(pose.x, xPosition)
//            val yPower = state.vPidY.calculate(pose.y, yPosition)
            val (vel, ang) = odometry.globalVelocity

//            state.vPidX.p = xvPIDPMap[vel.x]
//            val xPower = state.vPidX.calculate(xVelocity, vel.x)
//
//            state.vPidY.p = yvPIDPMap[vel.y]
//            val yPower = state.vPidY.calculate(yVelocity, vel.y)
//
//            state.vPidRot.p = rvPIDPMap[ang.radians]
//            val rotPower = state.vPidRot.calculate(rotVelocity, ang.radians)

            val xPower = state.pidX.calculate(xPosition, pose.x) * (((state.controlHub.getVoltage().coerceIn(10.0, 13.0) - 10) / 3) * .3 + .7)
            val yPower = state.pidY.calculate(yPosition, pose.y) * (((state.controlHub.getVoltage().coerceIn(10.0, 13.0) - 10) / 3) * .3 + .7)
            val rotPower = state.pidRot.calculate(rotPosition, pose.radians) * (((state.controlHub.getVoltage().coerceIn(10.0, 13.0) - 10) / 3) * .3 + .7)

//            println("xV: $xVelocity, yV: $yVelocity, xP: $xPower, yP: $yPower")

            val powers = fieldCalculatePowers(xPower, -yPower, -rotPower, odometry.globalPose.radians)

            val modifier = 1
            state.motors.setPower(powers[0] * modifier, powers[1] * modifier, powers[2] * modifier, powers[3] * modifier)
        }

        fun meow(state: DrivetrainState, odometry: OdometrySystem, targetPose: Pose? = null) {
            val pose = odometry.globalPose
            val velocity = odometry.globalVelocity
            val xVelocity = 0.0
            val yVelocity = 0.0
            val rotVelocity = Angle.degrees(200.0).radians
//
            val dashboard = FtcDashboard.getInstance()
            val dashboardTelemetry = dashboard.telemetry
//            if (targetPose != null) {
//            }

//            val xPosition = starting.x + sin(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)
//            val yPosition = starting.y + cos(beta) * state.profileDist!!.getPosition(state.timeInScheduler - state.timeStarted)
//            val rotPosition = state.profileRot!!.getPosition(state.timeInScheduler - state.timeStarted)

            // position PIDs, output is velocity
//            val xVelocity = state.pPidX.calculate(xPosition, pose.x)
//            val yVelocity = state.pPidY.calculate(yPosition, pose.y)
//            val rotVelocity = state.pPidRot.calculate(rotPosition, pose.radians)

//            println("xVel: $xVelocity")
//            println("yVel: $yVelocity")
//            println("rotVel: $rotVelocity")

//            val xPower = state.vPidX.calculate(pose.x, xPosition)
//            val yPower = state.vPidY.calculate(pose.y, yPosition)
//            val rotPower = state.vPidRot.calculate(state.profileRot!!.getPosition(state.timeInScheduler - state.timeStarted), pose.radians)

//            // velocity PIDs, output is power
            val (vel, ang) = odometry.globalVelocity

            state.vPidX.p = xvPIDPMap[vel.x]
            val xPower = state.vPidX.calculate(xVelocity, vel.x)

            state.vPidY.p = yvPIDPMap[vel.y]
            val yPower = state.vPidY.calculate(yVelocity, vel.y)

            state.vPidRot.p = rvPIDPMap[ang.radians]
            val rotPower = state.vPidRot.calculate(rotVelocity, ang.radians)

            println("v####: $vel")
            dashboardTelemetry.addData("xVPV", vel.x)
            dashboardTelemetry.addData("yVPV", vel.y)
            dashboardTelemetry.addData("rVPV", ang.radians)
            dashboardTelemetry.addData("xPSP", xVelocity)
            dashboardTelemetry.addData("yPSP", yVelocity)
            dashboardTelemetry.addData("rPSP", rotVelocity)
            dashboardTelemetry.addData("yPower", yPower)
            dashboardTelemetry.update()

            println("xV: $xVelocity, yV: $yVelocity, xP: $xPower, yP: $yPower")

            val powers = fieldCalculatePowers(0.0, -0.0, -rotPower, odometry.globalPose.radians)

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
            val compare = Pose(15, 15, 7.5)
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
        Mecanum.newMoveToPosition(beforeRun.state, odometrySystem, pose)
        beforeRun.state.moveToPositionTimeout = timeout
        setMode(ControlMode.AUTONOMOUS)
    }

    fun logMotorPowers(telemetry: Telemetry) {
//        for (motor in motors)
//            telemetry.addData(motor.deviceName, motor.power)
    }
}