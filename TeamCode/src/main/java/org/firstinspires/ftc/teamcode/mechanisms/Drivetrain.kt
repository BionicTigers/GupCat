package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.command.ConditionalCommand
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

private fun lerp(start: Double, end: Double, t: Double): Double {
    return start + (end - start) * t
}

class Drivetrain(hardwareMap: HardwareMap, private val robot: Robot) {
    private val motors: HashMap<String, DcMotorEx> = hashMapOf(
        "frontLeft" to hardwareMap.get(DcMotorEx::class.java, "frontLeft"),
        "frontRight" to hardwareMap.get(DcMotorEx::class.java, "frontRight"),
        "backLeft" to hardwareMap.get(DcMotorEx::class.java, "backLeft"),
        "backRight" to hardwareMap.get(DcMotorEx::class.java, "backRight")
    )

    private var velocity = 0.0

    init {
        for (motor in motors.values) {
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motors["frontLeft"]?.direction = DcMotorSimple.Direction.REVERSE
        motors["backLeft"]?.direction = DcMotorSimple.Direction.REVERSE
    }

    //Robot Centric - Determine Motor Powers
    fun robotDMP(pos: Vector2, mod: Double, turn: Double = 0.0) {
        println(pos)
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Double> = HashMap(4)
        velocity = lerp(velocity, max(pos.magnitude(), abs(turn)), Scheduler.deltaTime * 2)

        //Finds the ratio to scale the motor powers to
        val ratio: Double = max(abs(pos.x) + abs(pos.y) + abs(turn), 1.0)

        setPowers["frontLeft"] = velocity * (pos.y - pos.x + turn)
        setPowers["frontRight"] = velocity * (pos.y + pos.x - turn)
        setPowers["backLeft"] = velocity * (pos.y + pos.x + turn)
        setPowers["backRight"] = velocity * (pos.y - pos.x - turn)

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio) }
    }

    fun robotDMP(pos: Vector2, turn: Double = 0.0) {
        robotDMP(pos, 1.0, turn)
    }

    //Field Centric - Determine Motor Powers
    fun fieldDMP(pos: Vector2, mod: Double, turn: Double = 0.0) {
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Double> = HashMap(4)

        val heading: Double = robot.pose.rotation * PI / 180

        //Defines the movement direction
        val angleX: Double = pos.x * cos(heading) - pos.y * sin(heading)
        val angleY: Double = pos.x * sin(heading) + pos.y * cos(heading)

        //Finds the ratio to scale the motor powers to
        val ratio: Double = max(abs(angleX) + abs(angleY) + abs(turn), 1.0)

        setPowers["frontLeft"] = mod * (angleY - angleX + turn)
        setPowers["frontRight"] = mod * (angleY + angleX - turn)
        setPowers["backLeft"] = mod * (angleY + angleX + turn)
        setPowers["backRight"] = mod * (angleY - angleX - turn)

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio) }
    }

    fun fieldDMP(pos: Vector2, turn: Double = 0.0) {
        fieldDMP(pos, 1.0, turn)
    }

    fun moveToPosition(target: Pose): ConditionalCommand {
        val xPid = PID(PIDTerms(), 0.0, 3657.6, -1.0, 1.0)
        val yPid = PID(PIDTerms(), 0.0, 3657.6, -1.0, 1.0)
        val rPid = PID(PIDTerms(), -3600.0, 3600.0, -1.0, 1.0)

        return ConditionalCommand( {
            val error = Pose(
                xPid.calculate(target.x, robot.pose.x),
                yPid.calculate(target.y, robot.pose.y),
                rPid.calculate(target.rotation, robot.pose.rotation),
            )

            val magnitude = error.extractPosition().magnitude()
            val heading = atan2(error.x, error.y)

            val x = cos(heading - robot.pose.radians) * magnitude
            val y = sin(heading - robot.pose.radians) * magnitude

            val power = hypot(-x, y)
            val angle = atan2(y, -x)

            val angleSin = power * sin(angle)
            val angleCos = power * cos(angle)

            val setPowers: HashMap<String, Double> = HashMap(4)

            setPowers["frontLeft"] = angleSin - angleCos + error.rotation
            setPowers["frontRight"] = angleSin + angleCos - error.rotation
            setPowers["backLeft"] = angleSin + angleCos + error.rotation
            setPowers["backRight"] = angleSin - angleCos - error.rotation

            var highest = 0.0
            setPowers.forEach { (_, value) -> highest = if (highest < value) value else highest }
            setPowers.forEach { (name, value) -> motors[name]!!.power = (value / highest) }
        }, {
            val diff = robot.pose - target
            val compare = Pose(5.0, 5.0, 5.0)
            if (diff <= compare && diff >= -compare) false else this.stop(); true
        })
    }

    fun setup() {
        val (gamepad1, _) = robot.getGamepads()
        val left = gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        val right = gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        Scheduler.add(ContinuousCommand {
            fieldDMP(left.state!!, right.state!!.x)
        })
    }
    fun driveFrontLeft(){
        motors["frontLeft"]?.power = 1.0
    }

    fun driveFrontRight(){
        motors["frontRight"]?.power = 1.0
    }

    fun driveBackLeft(){
        motors["backLeft"]?.power = 1.0
    }

    fun driveBackRight(){
        motors["backRight"]?.power = 1.0
    }

    fun stop() {
        for(motor in motors.values) {
            motor.power = 0.0
        }
    }

    override fun toString(): String {
        return "Drivetrain { Front { ${motors["frontLeft"]}, ${motors["frontRight"]} }, Back { ${motors["backLeft"]}, ${motors["backRight"]} } }"
    }
}