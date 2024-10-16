package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.axiom.commands.System
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad

class Claw(hardwareMap: HardwareMap) : System {
    override val dependencies: List<System> = emptyList()
    override val beforeRun = null
    override val afterRun = null

    private val claw = hardwareMap.get(Servo::class.java, "claw")

    fun setupDriverControl(gp: Gamepad) {
        gp.getBooleanButton(Gamepad.Buttons.LEFT_BUMPER).onDown { open() }
        gp.getBooleanButton(Gamepad.Buttons.RIGHT_BUMPER).onDown { open() }
    }

    fun open() {
        claw.position = 1.0
    }

    fun close() {
        claw.position = 0.0
    }
}