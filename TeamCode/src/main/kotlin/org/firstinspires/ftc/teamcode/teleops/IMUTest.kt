//package org.firstinspires.ftc.teamcode.teleops
//
//import com.qualcomm.hardware.bosch.BNO055IMU
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation
//import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
//
//
//@TeleOp(name = "IMUTest")
//class IMUTest : LinearOpMode() {
//    override fun runOpMode() {
//
//        val parameters = BNO055IMU.Parameters()
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
//
//        val imu = hardwareMap.get(BNO055IMU::class.java, "externalIMU")
//        imu.initialize(parameters)
//
//        waitForStart()
//
//        while (opModeIsActive()) {
//            telemetry.addData("Heading", getRawHeading(imu))
//            telemetry.update()
//        }
//
//        Scheduler.clear()
//    }
//
//    fun getRawHeading(imu: BNO055IMU): Float {
//        val angles: Orientation =
//            imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
//        return angles.firstAngle
//    }
//}