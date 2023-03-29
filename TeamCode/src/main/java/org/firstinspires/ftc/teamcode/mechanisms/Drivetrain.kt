package com.atk

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Vector2
import kotlin.collections.HashMap
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

    //Robot Centric - Determine Motor Powers
    fun robotDMP(pos: Vector2, mod: Float, turn: Float = 0f) {
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Float> = HashMap(4)

        val heading: Float = robot.pose.rotation

        //Finds the ratio to scale the motor powers to
        val ratio: Float = max(abs(pos.x) + abs(pos.y) + abs(turn), 1f)

        setPowers["frontLeft"] = mod * (pos.y + pos.x + turn)
        setPowers["frontRight"] = mod * (pos.y - pos.x - turn)
        setPowers["backLeft"] = mod * (pos.y - pos.x + turn)
        setPowers["backRight"] = mod * (pos.y + pos.x - turn)

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio).toDouble() }
    }

    fun robotDMP(pos: Vector2, turn: Float = 0f) {
        robotDMP(pos, pos.magnitude(), turn)
    }

    //Field Centric - Determine Motor Powers
    fun fieldDMP(pos: Vector2, mod: Float, turn: Float = 0f) {
        //Create Motor Powers HashMap
        val setPowers: HashMap<String, Float> = HashMap(4)

        val heading: Float = robot.pose.rotation

        //Defines the movement direction
        val angleX: Float = pos.x * cos(heading) - pos.y * sin(heading)
        val angleY: Float = pos.x * sin(heading) + pos.y * cos(heading)

        //Finds the ratio to scale the motor powers to
        val ratio: Float = max(abs(angleX) + abs(angleY) + abs(turn), 1f)

        setPowers["frontLeft"] = mod * (angleY + angleX + turn)
        setPowers["frontRight"] = mod * (angleY - angleX - turn)
        setPowers["backLeft"] = mod * (angleY - angleX + turn)
        setPowers["backRight"] = mod * (angleY + angleX - turn)

        //Set motor powers scaled to the ratio
        setPowers.forEach { (name, value) -> motors[name]!!.power = (value / ratio).toDouble() }
    }

    fun fieldDMP(pos: Vector2, turn: Float = 0f) {
        fieldDMP(pos, pos.magnitude(), turn)
    }

    override fun toString(): String {
        return "Front { ${motors["frontLeft"]}, ${motors["frontRight"]} }\nBack { ${motors["backLeft"]}, ${motors["backRight"]} }"
    }
}