package org.firstinspires.ftc.teamcode.motion

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.utils.assignTracker
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.getTracker

@Config
object PIDCoef {
    @JvmField
    var p = 1.0
    @JvmField
    var i = 0.0
    @JvmField
    var d = 0.0
}

@TeleOp(name = "VelocityTune")
class VelocityTune : LinearOpMode() {
    override fun runOpMode() {
        val motor: DcMotorEx = hardwareMap.getByName("motor")
        val pid = PID(PIDCoef.p, PIDCoef.i, PIDCoef.d, 0.0, 2000.0, -1.0, 1.0)

        val dashboard = FtcDashboard.getInstance().telemetry
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        dashboard.addData("pv", motor.currentPosition)
        dashboard.addData("sp", 1000.0)
        dashboard.addData("output", motor.power * 100)
        dashboard.update()

        waitForStart()

        Scheduler.add(statelessCommand().setAction {
            motor.power = pid.calculate(1000.0, motor.velocity)
//            motor.power = 1.0
            dashboard.addData("pv", motor.velocity)
            dashboard.addData("sp", 1000.0)
            dashboard.addData("output", motor.power * 100)
            dashboard.update()
        })

        while (opModeIsActive()) {
            Scheduler.update()
        }
    }
}