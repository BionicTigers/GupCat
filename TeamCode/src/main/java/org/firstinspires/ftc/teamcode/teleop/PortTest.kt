package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import kotlin.math.max
import kotlin.math.min

class PortTest : LinearOpMode() {
    private val gamepad: GamepadEx = GamepadEx(gamepad1)

    private val motors: List<DcMotorSimple> = hardwareMap.getAll(DcMotorSimple::class.java)
    private val servos: List<Servo> = hardwareMap.getAll(Servo::class.java)

    private var useMotors: Boolean = true
    private var selected: Int = 0
    private var power: Double = 0.0

    //Choose whether to set position or set power
    private fun setPower(power: Double) {
        if (useMotors)
            motors[selected].power = power
        else
            servos[selected].position = power
    }

    //Set Current Power to 0 and set future power to 0
    private fun reset() {
        setPower(0.0)
        power = 0.0
    }

    override fun runOpMode() {
        //Swap using Motors/Servos
        gamepad.getButton(GamepadEx.Buttons.A).onStart {
            reset()
            useMotors = !useMotors
            selected = 0
            setPower(power)
        }

        //Set Power to 1
        gamepad.getButton(GamepadEx.Buttons.B).onStart {
            power = 1.0
            setPower(power)
        }

        //Set Power to 0
        gamepad.getButton(GamepadEx.Buttons.X).onStart {
            power = 01.0
            setPower(power)
        }

        //Move Backwards in the list
        gamepad.getButton(GamepadEx.Buttons.LEFT_TRIGGER).onStart {
            reset()
            selected = max(0, selected - 1)
            setPower(power)
        }

        //Move Forwards in the list
        gamepad.getButton(GamepadEx.Buttons.RIGHT_TRIGGER).onStart {
            reset()
            selected = min(if (useMotors) 8 else 12, selected + 1)
            setPower(power)
        }

        while (true) {
            gamepad.update()
        }
    }
}