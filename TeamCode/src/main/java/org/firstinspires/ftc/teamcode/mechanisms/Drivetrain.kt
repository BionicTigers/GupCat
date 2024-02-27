package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.command.Command
import org.firstinspires.ftc.teamcode.utils.command.CommandGroup
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.generateMotionProfile
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

private fun lerp(start: Double, end: Double, t: Double): Double {
    return start + (end - start) * t
}

class Drivetrain(hardwareMap: HardwareMap, private val robot: Robot) {
    private val hub = ControlHub(hardwareMap, "Control Hub")
    private val dashboard = FtcDashboard.getInstance()
    private val dashTelemetry = dashboard.telemetry

    private val motors: HashMap<String, DcMotorEx> = hashMapOf(
        "frontLeft" to hardwareMap.get(DcMotorEx::class.java, "frontLeft"),
        "frontRight" to hardwareMap.get(DcMotorEx::class.java, "frontRight"),
        "backLeft" to hardwareMap.get(DcMotorEx::class.java, "backLeft"),
        "backRight" to hardwareMap.get(DcMotorEx::class.java, "backRight")
    )

//    private val fl = .87
//    private val fr = 1.0
//    private val bl = .87
//    private val br = .82

    private val fl = 1.0
    private val fr = 1.0
    private val bl = 1.0
    private val br = 1.0

    private var x = 0.0
    private var y = 0.0
    private var rot = 0.0

    private var velocity = 0.0
    private var isInSlowMode = false

    init {
        for (motor in motors.values) {
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motors["frontLeft"]?.direction = DcMotorSimple.Direction.REVERSE
        motors["backLeft"]?.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setJunkPosition() {
        x = robot.pose.x
        y = robot.pose.y
        rot = robot.pose.radians
    }

    //Robot Centric - Determine Motor Powers
    fun robotDMP(pos: Vector2, mod: Double, turn: Double = 0.0) {
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Double> = HashMap(4)
        velocity =
            lerp(velocity, max(pos.magnitude(), abs(turn)), Scheduler.deltaTime.seconds() * 2)

        //Finds the ratio to scale the motor powers to
        val ratio: Double = max(abs(pos.x) + abs(pos.y) + abs(turn), 1.0)

        setPowers["frontLeft"] = (velocity * (pos.y - pos.x + turn)) * fl
        setPowers["frontRight"] = (velocity * (pos.y + pos.x - turn)) * fr
        setPowers["backLeft"] = (velocity * (pos.y + pos.x + turn)) * bl
        setPowers["backRight"] = (velocity * (pos.y - pos.x - turn)) * br

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio) }
    }

    fun robotDMP(pos: Vector2, turn: Double = 0.0) {
        robotDMP(pos, 1.0, turn)
    }

    //Field Centric - Determine Motor Powers
    fun fieldDMP(pos: Vector2, modVal: Double, turn: Double = 0.0) {
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Double> = HashMap(4)
        var mod = modVal

        val heading: Double = robot.pose.radians - rot

        //Defines the movement direction
        val angleX: Double = (pos.x * cos(-heading) - pos.y * sin(-heading))
        val angleY: Double = pos.x * sin(-heading) + pos.y * cos(-heading)

        //Finds the ratio to scale the motor powers to
        val ratio: Double = max(abs(angleX) + abs(angleY) + abs(turn), 1.0)
        if (hub.getVoltage() <= 8.0)
            mod *= .5

        if (isInSlowMode)
            mod *= .75

        mod *= -1

        val rotM = 1.0

        setPowers["frontLeft"] = (mod * (angleY - angleX + turn * rotM)) * fl
        setPowers["frontRight"] = (mod * (angleY + angleX - turn * rotM)) * fr
        setPowers["backLeft"] = (mod * (angleY + angleX + turn * rotM)) * bl
        setPowers["backRight"] = (mod * (angleY - angleX - turn * rotM)) * br

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio) }
    }

    fun fieldDMP(pos: Vector2, turn: Double = 0.0) {
        fieldDMP(Vector2(-pos.x, pos.y), 1.0, -turn)
    }

    fun moveToPosition(target: Pose): Command {
        println("x: ${robot.pose.x} ${target.x}")
        println("y: ${robot.pose.y} ${target.y}")
        val xPid = PID(PIDTerms(30.0), 0.0, 3600.0, -1.0, 1.0)
        val yPid = PID(PIDTerms(30.0), 0.0, 3600.0, -1.0, 1.0)
        val rPid = PID(PIDTerms(10.0), -360.0, 360.0, -360.0, 360.0)

        val xProfile = generateMotionProfile(
            robot.pose.x,
            target.x,
            3000.0,
            7000.0,
            4000.0
        ) //TODO get correct mv
        val yProfile = generateMotionProfile(
            robot.pose.y,
            target.y,
            3000.0,
            7000.0,
            4000.0
        ) //TODO get correct mv

        return CommandGroup()
            .add(Command({
                val setPowers: HashMap<String, Double> = HashMap(4)

                val error = Pose(
                    xPid.calculate(xProfile.getPosition(it.elapsedTime), robot.pose.x),
                    yPid.calculate(yProfile.getPosition(it.elapsedTime), robot.pose.y),
                    rPid.calculate(target.rotation, robot.pose.rotation)
                )

                dashTelemetry.addData("xPV", robot.pose.x)
                dashTelemetry.addData("yPV", robot.pose.y)
                dashTelemetry.addData("xSP", xProfile.getPosition(it.elapsedTime))
                dashTelemetry.addData("ySP", yProfile.getPosition(it.elapsedTime))
                dashTelemetry.addData("xActualAccel", robot.acceleration.x)
                dashTelemetry.addData("yActualAccel", robot.acceleration.y)
                dashTelemetry.addData("xCommandedAccel", xProfile.getAcceleration(it.elapsedTime))
                dashTelemetry.addData("yCommandedAccel", yProfile.getAcceleration(it.elapsedTime))
                dashTelemetry.addData("xActualVelocity", robot.velocity.x)
                dashTelemetry.addData("yActualVelocity", robot.velocity.y)
                dashTelemetry.addData("xCommandedVelocity", xProfile.getVelocity(it.elapsedTime))
                dashTelemetry.addData("yCommandedVelocity", yProfile.getVelocity(it.elapsedTime))
                dashTelemetry.update()

                fieldDMP(Vector2(error.x, error.y), -error.rotation * 1.5)
            }, {
                val diff = (robot.pose - target).abs()
                val compare = Pose(20.0, 20.0, 5.0)
                return@Command diff >= compare || (diff.rotation >= compare.rotation || diff.rotation <= -compare.rotation)
            }))
            .add(Command {
                this.stop()
            })
            .build()
    }

    fun setup() {
        val (gamepad1, _) = robot.getGamepads()
        val left = gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        val right = gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        Scheduler.add(continuousCommand {
            fieldDMP(left.state, right.state.x)
        })
    }

    fun driveFrontLeft() {
        motors["frontLeft"]?.power = 1.0 * fl
    }

    fun driveFrontRight() {
        motors["frontRight"]?.power = 1.0 * fr
    }

    fun driveBackLeft() {
        motors["backLeft"]?.power = 1.0 * bl
    }

    fun driveBackRight() {
        motors["backRight"]?.power = 1.0 * br
    }

    fun stop() {
        for (motor in motors.values) {
            motor.power = 0.0
        }
    }

    fun slowToggle() {
        isInSlowMode = !isInSlowMode
    }


    override fun toString(): String {
        return "Drivetrain { Front { ${motors["frontLeft"]}, ${motors["frontRight"]} }, Back { ${motors["backLeft"]}, ${motors["backRight"]} } }"
    }
}