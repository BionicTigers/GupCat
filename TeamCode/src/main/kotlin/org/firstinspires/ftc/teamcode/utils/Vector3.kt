package org.firstinspires.ftc.teamcode.utils

class Vector3(val x: Double, val y: Double, val z: Double) {
    constructor(x: Number, y: Number, z: Number) : this(x.toDouble(), y.toDouble(), z.toDouble())
    constructor(): this(0.0, 0.0, 0.0)

    operator fun plus(other: Vector3): Vector3 {
        return Vector3(x + other.x, y + other.y, z + other.z)
    }

    operator fun minus(other: Vector3): Vector3 {
        return Vector3(x - other.x, y - other.y, z - other.z)
    }

    operator fun times(scalar: Double): Vector3 {
        return Vector3(x * scalar, y * scalar, z * scalar)
    }

    operator fun div(scalar: Double): Vector3 {
        return Vector3(x / scalar, y / scalar, z / scalar)
    }

    fun dot(other: Vector3): Double {
        return x * other.x + y * other.y + z * other.z
    }

    fun cross(other: Vector3): Vector3 {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        )
    }

    fun magnitude(): Double {
        return Math.sqrt(x * x + y * y + z * z)
    }

    fun normalize(): Vector3 {
        return this / magnitude()
    }

    fun distanceTo(other: Vector3): Double {
        return (this - other).magnitude()
    }

    fun angleTo(other: Vector3): Double {
        return Math.acos(dot(other) / (magnitude() * other.magnitude()))
    }

    fun projectOnto(other: Vector3): Vector3 {
        return other * (dot(other) / other.magnitude())
    }

    fun rejectFrom(other: Vector3): Vector3 {
        return this - projectOnto(other)
    }

    fun rotateAround(axis: Vector3, angle: Double): Vector3 {
        val cos = Math.cos(angle)
        val sin = Math.sin(angle)
        val projection = projectOnto(axis)
        val rejection = rejectFrom(axis)
        return projection + rejection * cos + axis.cross(rejection) * sin
    }

    override fun toString(): String {
        return "Vector3($x, $y, $z)"
    }

    companion object {
        val ZERO = Vector3(0.0, 0.0, 0.0)
        val X = Vector3(1.0, 0.0, 0.0)
        val Y = Vector3(0.0, 1.0, 0.0)
        val Z = Vector3(0.0, 0.0, 1.0)
    }
}