package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.axiom.commands.System
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad
import org.firstinspires.ftc.teamcode.utils.getByName

class Claw(hardwareMap: HardwareMap) : System {
    enum class Sample {
        YELLOW,
        BLUE,
        RED
    }

    override val dependencies: List<System> = emptyList()
    override val beforeRun = null
    override val afterRun = null

    private val claw = hardwareMap.getByName<Servo>("claw")

    fun setupDriverControl(gp: Gamepad) {
        gp.getBooleanButton(Gamepad.Buttons.LEFT_BUMPER).onDown { open() }
        gp.getBooleanButton(Gamepad.Buttons.RIGHT_BUMPER).onDown { close() }
    }

    fun open() {
        claw.position = 0.0
    }

    fun close() {
        claw.position = 0.6
    }

//    fun getDetection(): Sample {
//
//    }

    fun logclaw(telemetry: Telemetry) {
        telemetry.addData("position", claw.position)
        telemetry.update()
    }
}