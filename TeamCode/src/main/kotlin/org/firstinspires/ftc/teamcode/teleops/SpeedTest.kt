package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.ControlHub
import io.github.bionictigers.axiom.utils.Time

@TeleOp(name = "Speed")
class SpeedTest : LinearOpMode() {
    override fun runOpMode() {
//        val motor: DcMotorEx = hardwareMap.getByName("backRight")
//        val motor2: DcMotorEx = hardwareMap.getByName("backLeft")
//        val motor3: DcMotorEx = hardwareMap.getByName("frontLeft")

//        val servo: Servo = hardwareMap.getByName("servo :3")
        val controlHub = ControlHub(hardwareMap, "Control Hub")


        waitForStart()

        var allTimes = arrayListOf<Long>()
        var lastTime = System.nanoTime()
        while (opModeIsActive()) {
//            motor.power = Random.nextDouble()
//            motor2.power = Random.nextDouble()
//            motor3.power = Random.nextDouble()
//            servo.position = Random.nextDouble()
            controlHub.refreshBulkData()
            controlHub.getEncoderTicks(0)
            controlHub.getEncoderTicks(3)
            allTimes.add(System.nanoTime() - lastTime)
            lastTime = System.nanoTime()
        }
        val sum = allTimes.sum()
        println(Time.fromNanoseconds(sum / allTimes.size).milliseconds())
    }
}