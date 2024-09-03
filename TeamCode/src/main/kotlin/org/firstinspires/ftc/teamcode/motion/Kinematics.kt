package org.firstinspires.ftc.teamcode.motion

import org.firstinspires.ftc.teamcode.utils.Matrix

/**
 * Kinematics for a mecanum drive
 *
 * @param wheelRadius The radius of the wheel in millimeters
 * @param wheelBase The distance between the front and back wheels in millimeters
 * @param trackWidth The distance between the left and right wheels in millimeters
 */
data class MecanumKinematics(val wheelRadius: Double, val wheelBase: Double, val trackWidth: Double) {
    var inverseMatrix = Matrix(4, 3)

    init {
        val sum = wheelBase + trackWidth

        inverseMatrix[0] = arrayOf(1.0, -1.0, -sum)
        inverseMatrix[1] = arrayOf(1.0, 1.0, sum)
        inverseMatrix[2] = arrayOf(1.0, 1.0, -sum)
        inverseMatrix[3] = arrayOf(1.0, -1.0, sum)

        inverseMatrix *= 1 / wheelRadius
    }

    fun inverse(xVelocity: Double, yVelocity: Double, rotation: Double): Array<Double> {
        val input = Matrix(3, 1)
        input[0] = arrayOf(xVelocity)
        input[1] = arrayOf(yVelocity)
        input[2] = arrayOf(rotation)

        val output = inverseMatrix * input

        return arrayOf(output[0][0], output[1][0], output[2][0], output[3][0])
    }
}