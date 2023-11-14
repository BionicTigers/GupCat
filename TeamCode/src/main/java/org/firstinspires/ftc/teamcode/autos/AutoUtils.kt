package org.firstinspires.ftc.teamcode.autos

import org.firstinspires.ftc.teamcode.utils.Pose

val Offsets = mapOf(
    "center" to Pose(2731.0, 950.0, 0.0),
    "left" to Pose(2462.0, 835.0, 0.0),
    "right" to Pose(2993.0, 835.0, 0.0)
)

enum class Detection {
    Left,
    Center,
    Right
}