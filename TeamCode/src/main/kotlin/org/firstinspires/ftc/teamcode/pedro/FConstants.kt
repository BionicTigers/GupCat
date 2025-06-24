package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.Localizers
import com.qualcomm.robotcore.hardware.DcMotorSimple

object FConstants {
    init {
        FollowerConstants.leftFrontMotorName = "frontLeft"
        FollowerConstants.leftRearMotorName = "backLeft"
        FollowerConstants.rightFrontMotorName = "frontRight"
        FollowerConstants.rightRearMotorName = "backRight"
        FollowerConstants.mass = 14.651
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE
    }
}