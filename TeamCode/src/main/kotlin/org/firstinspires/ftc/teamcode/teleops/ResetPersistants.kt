package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.Persistents

@TeleOp(name = "ResetPersistants")
class ResetPersistants : LinearOpMode() {
    override fun runOpMode() {
        Persistents.reset()
    }
}