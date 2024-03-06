package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utils.ControlHub

/** Runs a tape measure up to the rigging in order to lift the robot */
class Hang (hardwareMap: HardwareMap) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, "hangMotor")
    private val hub = ControlHub(hardwareMap, "Control Hub")

    enum class HangState {
        Pull,
        Raise,
        Stop
    }

    var hangState = HangState.Stop

    init {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        hub.setJunkTicks()
    }

    /** Toggle the hanging mechanism to run at 100% power */
    fun pull() {
        if (hangState != HangState.Pull) {
            hangState = HangState.Pull
            motor.power = 1.0
        } else stop()
    }

    /** Runs the hanging mechanism down at 100% power */
    fun raise() {
        if (hangState != HangState.Raise) {
            hangState = HangState.Raise
            motor.power = -1.0
        } else stop()
    }

    /** Stops the motor */
    fun stop() {
        hangState = HangState.Stop
        motor.power = 0.0
    }
}