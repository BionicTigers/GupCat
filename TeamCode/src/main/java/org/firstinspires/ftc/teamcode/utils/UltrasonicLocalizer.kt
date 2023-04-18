package org.firstinspires.ftc.teamcode.utils
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.UltrasonicSensor
import kotlin.math.atan
import kotlin.math.cos

class UltrasonicLocalizer (hardwareMap: HardwareMap) {
    // measurements in cm
    private val lateralOffset = 4.0
    private val sideOffset = 4.0
    private val backOffset = 4.0

    private val side1 = hardwareMap.get(UltrasonicSensor::class.java, "side1")
    private val side2 = hardwareMap.get(UltrasonicSensor::class.java, "side2")
    private val back = hardwareMap.get(UltrasonicSensor::class.java, "back")

    fun update(): Pose {
        // measurements in cm
        val side1Dist = side1.ultrasonicLevel + sideOffset
        val side2Dist = side2.ultrasonicLevel + sideOffset
        val backDist = back.ultrasonicLevel + backOffset

        // slope of the wall compared to the robot
        val slope = (side2Dist - side1Dist) / lateralOffset
        // changes the slope into the angle of the robot
        val angle = atan(slope)

        // robot position
        val x = cos(angle) * (side1Dist + (side1Dist - side2Dist))
        val y = cos(angle) * backDist

        return Pose(x, y, angle)
    }
}