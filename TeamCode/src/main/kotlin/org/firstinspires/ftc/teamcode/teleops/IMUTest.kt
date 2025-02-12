package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.hardware.bosch.BNO055IMUNew
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.utils.getByName


@TeleOp(name = "IMUTest")
class IMUTest : LinearOpMode() {
    override fun runOpMode() {

        val imu = hardwareMap.getByName<BNO055IMUNew>("InternalIMU")
        imu.initialize(BNO055IMUNew.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Heading", getRawHeading(imu))
            telemetry.update()
        }

        Scheduler.clear()
    }

    fun getRawHeading(imu: BNO055IMUNew): Float {
        val angles: Orientation =
            imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        return angles.firstAngle
    }
}