package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.command.ConditionalCommand
import org.firstinspires.ftc.teamcode.utils.command.ContinuousCommand
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.generateMotionProfile
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

//    private val fl = .87
//    private val fr = 1.0
//    private val bl = .87
//    private val br = .82

    private val fl = 1.0
    private val fr = 1.0
    private val bl = 1.0
    private val br = 1.0

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
    fun fieldDMP(pos: Vector2, mod: Double, turn: Double = 0.0) {
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Double> = HashMap(4)

        val heading: Double = robot.pose.radians

        //Defines the movement direction
        val angleX: Double = (pos.x * cos(-heading) - pos.y * sin(-heading))
        val angleY: Double = pos.x * sin(-heading) + pos.y * cos(-heading)

        //Finds the ratio to scale the motor powers to
        val ratio: Double = max(abs(angleX) + abs(angleY) + abs(turn), 1.0)

        setPowers["frontLeft"] = (mod * (angleY - angleX + turn) ) * fl
        setPowers["frontRight"] = (mod * (angleY + angleX - turn) ) * fr
        setPowers["backLeft"] = (mod * (angleY + angleX + turn) ) * bl
        setPowers["backRight"] = (mod * (angleY - angleX - turn) ) * br

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio) }
    }

    fun fieldDMP(pos: Vector2, turn: Double = 0.0) {
        fieldDMP(pos, 1.0, turn)
    }

    fun moveToPosition(target: Pose): ConditionalCommand {
        val xPid = PID(PIDTerms(1.0), 0.0, 3657.6, -1000.0, 1000.0)
        val yPid = PID(PIDTerms(1.0), 0.0, 3657.6, -1000.0, 1000.0)
        val rPid = PID(PIDTerms(1.0), -360.0, 360.0, -360.0, 360.0)

        val xProfile = generateMotionProfile(robot.pose.x, target.x, 30.0, 30.0, 30.0) //TODO get correct mv
        val yProfile = generateMotionProfile(robot.pose.y, target.y, 30.0, 30.0, 30.0) //TODO get correct mv

        return ConditionalCommand({
            val setPowers: HashMap<String, Double> = HashMap(4)

            val error = Pose(
                xPid.calculate(target.x, robot.pose.x),
                yPid.calculate(target.y, robot.pose.y),
                rPid.calculate(target.rotation, robot.pose.rotation),
            )

            val heading = atan2(error.x, -error.y)
            val angleError = Math.toRadians(robot.pose.rotation - target.rotation)
            val y = cos(heading)
            val x = sin(heading)

            val power = hypot(x, y)
            val angleVector = Vector2(x * sin(robot.pose.radians) + y * cos(robot.pose.radians), x * cos(robot.pose.radians) - y * sin(robot.pose.radians))

            setPowers["frontLeft"] = (power * (angleVector.x - angleVector.y) + angleError) * fl
            setPowers["frontRight"] = (power * (angleVector.x + angleVector.y) - angleError) * fr
            setPowers["backLeft"] = (power * (angleVector.x + angleVector.y) + angleError) * bl
            setPowers["backRight"] = (power * (angleVector.x - angleVector.y) - angleError) * br

            var highest = 0.0
            setPowers.forEach { (_, value) -> highest = if (highest < abs(value)) abs(value) else highest }
            highest *= 2
            setPowers.forEach { (name, value) -> motors[name]!!.power = (value / highest) }
        }, {
            val diff = robot.pose - target
            val compare = Pose(20.0, 20.0, 20.0)
            if (diff.abs() >= compare) {
                return@ConditionalCommand true
            } else {
                this.stop()
                return@ConditionalCommand false
            }
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
        motors["frontLeft"]?.power = 1.0 * fl
    }

    fun driveFrontRight(){
        motors["frontRight"]?.power = 1.0 * fr
    }

    fun driveBackLeft(){
        motors["backLeft"]?.power = 1.0 * bl
    }

    fun driveBackRight(){
        motors["backRight"]?.power = 1.0 * br
    }

    fun stop() {
        for(motor in motors.values) {
            motor.power = 0.0
        }
    }

    fun slow() {
        for(motor in motors.values) {
            motor.power = 0.75
        }
    }


    override fun toString(): String {
        return "Drivetrain { Front { ${motors["frontLeft"]}, ${motors["frontRight"]} }, Back { ${motors["backLeft"]}, ${motors["backRight"]} } }"
    }
}