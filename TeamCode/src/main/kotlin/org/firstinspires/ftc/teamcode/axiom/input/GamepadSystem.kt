package org.firstinspires.ftc.teamcode.axiom.input

import com.qualcomm.robotcore.hardware.Gamepad as FTCGamepad
import io.github.bionictigers.commands.System
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand

class GamepadSystem(gamepad1FTC: FTCGamepad, gamepad2FTC: FTCGamepad) : System {
    override val beforeRun = null
    override val afterRun = null

    override val dependencies: List<System> = emptyList()

    val gamepad1 = Gamepad(gamepad1FTC, this)
    val gamepad2 = Gamepad(gamepad2FTC, this)


}