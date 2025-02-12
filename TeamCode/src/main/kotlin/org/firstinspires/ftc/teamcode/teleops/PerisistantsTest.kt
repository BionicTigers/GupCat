package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.utils.Persistents

@TeleOp(name = "PersistantsTest")
class PersistantsTest : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        while (opModeIsActive()) {
            Persistents.log(telemetry)
            telemetry.update()
        }
    }
}