package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import io.github.bionictigers.axiom.commands.System
import org.firstinspires.ftc.teamcode.input.Gamepad
import org.firstinspires.ftc.teamcode.utils.getByName

//enum class Sample {
//    YELLOW,
//    BLUE,
//    RED
//}


class Claw(hardwareMap: HardwareMap, private val openPos: Double = .15) : System {
    override val dependencies: List<System> = emptyList()
    override val beforeRun = null
    override val afterRun = null

    private val claw = hardwareMap.getByName<Servo>("claw")

    fun setupDriverControl(gp: Gamepad) {
        gp.getBooleanButton(Gamepad.Buttons.A).onDown {
            open = !open
        }
    }

    private fun open() {
        position = openPos
    }

    private fun close() {
        position = 0.60
    }

    var position: Double
        get() = claw.position
        set(value) {
            claw.position = value.coerceIn(0.0, 1.0)
        }

    var open = false
        set(value) {
            if (value) open() else close()
            field = value
        }

//    fun getDetection(): Sample {
//
//    }

    fun logClaw(telemetry: Telemetry) {
        telemetry.addData("position", claw.position)
    }
}