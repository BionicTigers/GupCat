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
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class Drivetrain(hardwareMap: HardwareMap, private val robot: Robot) {
    private val motors: HashMap<String, DcMotorEx> = hashMapOf(
        "frontLeft" to hardwareMap.get(DcMotorEx::class.java, "frontLeft"),
        "frontRight" to hardwareMap.get(DcMotorEx::class.java, "frontRight"),
        "backLeft" to hardwareMap.get(DcMotorEx::class.java, "backLeft"),
        "backRight" to hardwareMap.get(DcMotorEx::class.java, "backRight")
    )

    init {
        for (motor in motors.values) {
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motors["frontRight"]?.direction = DcMotorSimple.Direction.REVERSE
        motors["backRight"]?.direction = DcMotorSimple.Direction.REVERSE
    }

    //Robot Centric - Determine Motor Powers
    fun robotDMP(pos: Vector2, mod: Double, turn: Double = 0.0) {
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Double> = HashMap(4)

        //Finds the ratio to scale the motor powers to
        val ratio: Double = max(abs(pos.x) + abs(pos.y) + abs(turn), 1.0)

        setPowers["frontLeft"] = mod * (pos.y + -pos.x + turn)
        setPowers["frontRight"] = mod * (pos.y - -pos.x - turn)
        setPowers["backLeft"] = mod * (pos.y - -pos.x + turn)
        setPowers["backRight"] = mod * (pos.y + -pos.x - turn)

        println(setPowers)

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

        val heading: Double = robot.pose.rotation

        //Defines the movement direction
        val angleX: Double = pos.x * cos(heading) - pos.y * sin(heading)
        val angleY: Double = pos.x * sin(heading) + pos.y * cos(heading)

        //Finds the ratio to scale the motor powers to
        val ratio: Double = max(abs(angleX) + abs(angleY) + abs(turn), 1.0)

        setPowers["frontLeft"] = mod * (angleY + angleX + turn)
        setPowers["frontRight"] = mod * (angleY - angleX - turn)
        setPowers["backLeft"] = mod * (angleY - angleX + turn)
        setPowers["backRight"] = mod * (angleY + angleX - turn)

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio) }
    }

    fun fieldDMP(pos: Vector2, turn: Double = 0.0) {
        fieldDMP(pos, pos.magnitude(), turn)
    }

    fun moveToPosition(target: Pose): ConditionalCommand {
        val xPid = PID(PIDTerms(), 0.0, 3657.6, -1.0, 1.0)
        val yPid = PID(PIDTerms(), 0.0, 3657.6, -1.0, 1.0)
        val rPid = PID(PIDTerms(), -360.0, 360.0, -1.0, 1.0)

        return ConditionalCommand({
            val current = robot.pose
            robotDMP(
                Vector2(
                    xPid.calculate(target.x, current.x),
                    yPid.calculate(target.y, current.y)
                ),
                rPid.calculate(target.rotation, current.rotation)
            )
        }, { (robot.pose - target).compare(5.0, 5.0, 5.0) })
    }

    fun setup() {
        val (gamepad1, _) = robot.getGamepads()
        val left = gamepad1.getJoystick(GamepadEx.Joysticks.LEFT_JOYSTICK)
        val right = gamepad1.getJoystick(GamepadEx.Joysticks.RIGHT_JOYSTICK)

        Scheduler.add(ContinuousCommand {
            robotDMP(left.state!!, right.state!!.x)
        })
    }

    override fun toString(): String {
        return "Drivetrain { Front { ${motors["frontLeft"]}, ${motors["frontRight"]} }, Back { ${motors["backLeft"]}, ${motors["backRight"]} } }"
    }
}