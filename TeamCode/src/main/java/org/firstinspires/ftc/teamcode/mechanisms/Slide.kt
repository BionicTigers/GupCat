package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.MotionResult
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import org.firstinspires.ftc.teamcode.utils.generateMotionProfile
import kotlin.math.floor

/**
 * Raises and lowers arm and chainbar for more versatile scoring positions
 */
class Slide(hardwareMap: HardwareMap) {
    val left = hardwareMap.get(DcMotorEx::class.java, "slideBack")
    val right = hardwareMap.get(DcMotorEx::class.java, "slideFront")
    private val pid = PID(PIDTerms(), 0.0, 1000.0, -1.0, 1.0)
    private val hub = ControlHub(hardwareMap, hardwareMap.get("Control Hub") as LynxDcMotorController)
    private val dashboard = FtcDashboard.getInstance()
    private val dashTelemetry = dashboard.telemetry

//    private lateinit var profile: MotionResult
    private lateinit var elapsedTime: ElapsedTime

    /**
     * Sets initial height of the slides to 0
     */
    var height = 0.0
        set(value) {
            field = value.coerceIn(0.0, 1450.0)
//            profile = generateMotionProfile(field, 8.5, 8.5, 20.0)
            elapsedTime = ElapsedTime()
        }

    /**
     * Initializes slides to make sure default settings are what is needed for proper function
     */
    init {
        hub.setJunkTicks() //Allows hub to ignore old encoder ticks
        right.direction = DcMotorSimple.Direction.REVERSE //Reverses one motor to prevent conflicts
        right.mode = DcMotor.RunMode.RUN_USING_ENCODER
        left.mode = DcMotor.RunMode.RUN_USING_ENCODER
        height = 0.0
    }

    /**
     * Calculates and sets power using position and PID algorithm
     */
    fun update() {
        hub.refreshBulkData()
        val encoderTicks = hub.getEncoderTicks(2).toDouble()
//        val targetHeight = profile.position.getOrElse(floor(elapsedTime.seconds() / profile.deltaTime).toInt()) { height }
        var power = pid.calculate(height, encoderTicks)
        if (encoderTicks > 50)
            power += .15

        left.power = power
        right.power = power
        dashTelemetry.addData("pv", encoderTicks)
        dashTelemetry.addData("sp", height)
        dashTelemetry.update()
    }
}