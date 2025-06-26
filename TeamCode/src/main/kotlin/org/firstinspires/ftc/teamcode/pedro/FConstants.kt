package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.Localizers
import com.qualcomm.robotcore.hardware.DcMotorSimple

object FConstants {
    init {
        FollowerConstants.localizers = Localizers.PINPOINT
        FollowerConstants.leftFrontMotorName = "frontLeft"
        FollowerConstants.leftRearMotorName = "backLeft"
        FollowerConstants.rightFrontMotorName = "frontRight"
        FollowerConstants.rightRearMotorName = "backRight"
        FollowerConstants.useBrakeModeInTeleOp = true
        FollowerConstants.mass = 14.651
        FollowerConstants.xMovement = 72.84
        FollowerConstants.yMovement = 50.35
        FollowerConstants.forwardZeroPowerAcceleration = -47.6
        FollowerConstants.lateralZeroPowerAcceleration = -87.35
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(.0625, .00005, 0.0, 0.0)
        FollowerConstants.headingPIDFCoefficients.setCoefficients(.7, 0.0, 0.0, 0.0)
        FollowerConstants.drivePIDFCoefficients.setCoefficients(.006, 0.0, 0.00001, 0.6, 0.0)
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD
    }
}