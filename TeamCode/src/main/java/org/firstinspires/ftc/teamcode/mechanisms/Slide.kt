package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms

/**
 * Raises and lowers arm and chainbar for more versatile scoring positions
 */
class Slide(hardwareMap: HardwareMap) {
    val left = hardwareMap.get(DcMotorEx::class.java, "slideBack")
    val right = hardwareMap.get(DcMotorEx::class.java, "slideFront")
    private val pid = PID(PIDTerms(), 0.0, 1000.0, -1.0, 1.0)
    private val hub = ControlHub(hardwareMap, hardwareMap.get("Control Hub") as LynxDcMotorController)

    /**
     * Sets initial height of the slides to 0
     */
    var height = 0.0
        set(value) {
            field = value.coerceIn(0.0, 1560.0)
        }

    /**
     * Initializes slides to make sure default settings are what is needed for proper function
     */
    init {
        hub.setJunkTicks() //Allows hub to ignore old encoder ticks
        right.direction = DcMotorSimple.Direction.REVERSE //Reverses one motor to prevent conflicts
        right.mode = DcMotor.RunMode.RUN_USING_ENCODER
        left.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    /**
     * Calculates and sets power using position and PID algorithm
     */
    fun update() {
        hub.refreshBulkData()
        val encoderTicks = hub.getEncoderTicks(2).toDouble()
        val power = pid.calculate(height, encoderTicks)
        left.power = power
        right.power = power
    }
}