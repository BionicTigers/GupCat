package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.teamcode.autos.CommandMove
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.getByName

interface CommandSFRASS : CommandState {
    @Editable
    var power: Double

    var velocity: Double

    companion object {
        fun default(): CommandSFRASS {
            return object : CommandSFRASS, CommandState by CommandState.default("Meow :3") {
                override var power: Double = 1.0
                override var velocity: Double = 0.0
            }
        }
    }
}

@TeleOp(name = "Test")
class Test : LinearOpMode() {
    override fun runOpMode() {
//        val pivot = Pivot(hardwareMap)
//        val gamepadSystem = GamepadSystem(gamepad1, gamepad2)
//        val (gp1, gp2) = gamepadSystem.gamepads

        val controlHub = ControlHub(hardwareMap, "Control Hub")
        val motor: DcMotorEx = hardwareMap.getByName("motor")
        var oldPosition = motor.currentPosition
        Scheduler.add(
            Command(CommandSFRASS.default())
                .setAction {
                    motor.power = it.power
                    controlHub.refreshBulkData()
                    val pos = controlHub.getEncoderTicks(3)
                    it.velocity = (pos - oldPosition).toDouble() / it.deltaTime.seconds()
                    oldPosition = pos
                    false
                }
        )

//        pivot.setupDriverControl(gp1)

        waitForStart()
        while (opModeIsActive()) {
            Scheduler.update()
        }
    }
}