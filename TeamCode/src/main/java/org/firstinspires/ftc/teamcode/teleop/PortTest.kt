package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import kotlin.math.max
import kotlin.math.min

@TeleOp(name="Port Test", group="Testing")
class PortTest : LinearOpMode() {
    private lateinit var gamepad: GamepadEx

    private lateinit var motors: List<DcMotorSimple>
    private lateinit var servos: List<Servo>

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
        gamepad = GamepadEx(gamepad1)
        motors = hardwareMap.getAll(DcMotorSimple::class.java)
        println(motors)
        servos = hardwareMap.getAll(Servo::class.java)
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
            power = 0.0
            setPower(power)
        }

        //Move Backwards in the list
        gamepad.getButton(GamepadEx.Buttons.LEFT_BUMPER).onStart {
            reset()
            selected = max(0, selected - 1)
            setPower(power)
        }

        //Move Forwards in the list
        gamepad.getButton(GamepadEx.Buttons.RIGHT_BUMPER).onStart {
            reset()
            selected = min(if (useMotors) motors.size-1 else servos.size-1, selected + 1)
            setPower(power)
        }

        waitForStart()

        while (opModeIsActive()) {
            gamepad.update()
            telemetry.addData("Type", if (useMotors) "Motor" else "Servo")
            telemetry.addData("Port", selected)
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }
}