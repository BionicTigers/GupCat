package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.System
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.motion.generateMotionProfile
import org.firstinspires.ftc.teamcode.utils.Time

class Arm(hardwareMap: HardwareMap) : System {
    override val dependencies: List<System> = emptyList()
    override val beforeRun = null
    override val afterRun = null

    private val arm = hardwareMap.get(Servo::class.java, "claw")
    fun setupDriverControl(gp: Gamepad) {
        gp.getBooleanButton(Gamepad.Buttons.LEFT_BUMPER).onDown { up() }
        gp.getBooleanButton(Gamepad.Buttons.RIGHT_BUMPER).onDown { down() }
    }

    var position: Double
        get() = arm.position
        set(value) {
            arm.position = value.coerceIn(0.0, 1.0)
        }

    fun up() {
        position = 1.0
    }

    fun down() {
        position = 0.0
    }
}