package org.firstinspires.ftc.teamcode.input

import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides


interface Profile {
    val pivot: Pivot.Schema
    val slides: Slides.Schema
    val arm: Arm.Schema
    val claw: Claw.Schema
}