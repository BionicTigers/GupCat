package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.MotionResult
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.PIDTerms
import kotlin.math.floor

/**
 * Raises and lowers arm and chainbar for more versatile scoring positions
 */
class Slide(hardwareMap: HardwareMap) {
    //TODO (Melia) Declare an enum with the different heights the slides can be in

    val left = hardwareMap.get(DcMotorEx::class.java, "slideBack")
    val limitSwitch = hardwareMap.get(DigitalChannel::class.java, "limitSwitch")
    private val hub = ControlHub(hardwareMap, "Control Hub")

    private val pid = PID(PIDTerms(), 0.0, 1000.0, -1.0, 1.0)
    private var profile: MotionResult? = null
    private lateinit var elapsedTime: ElapsedTime

    private val dashboard = FtcDashboard.getInstance()
    private val dashTelemetry = dashboard.telemetry

    /**
     * Sets initial height of the slides to 0
     */
    var height = 0.0
        set(value) {
            hub.refreshBulkData()
            field = value.coerceIn(-50.0, 1450.0)
//            profile = generateMotionProfile(hub.getEncoderTicks(2).toDouble(), field, 7000.0, 7000.0, 20000.0)
            elapsedTime = ElapsedTime()
        }





    /**
     * Initializes slides to make sure default settings are what is needed for proper function
     */
    init {
        hub.setJunkTicks() //Allows hub to ignore old encoder ticks
        left.mode = DcMotor.RunMode.RUN_USING_ENCODER
        left.direction = DcMotorSimple.Direction.REVERSE
        height = 0.0
    }

    /**
     * Calculates and sets power using position and PID algorithm
     */
    fun update() {
        hub.refreshBulkData()
//        if (!limitSwitch.state) {
//            hub.setJunkTicks()
//            height = 0.0
//        }
        val encoderTicks = hub.getEncoderTicks(0).toDouble()
        val targetHeight = if (profile != null) profile!!.position.getOrElse(floor(elapsedTime.seconds() / profile!!.deltaTime).toInt()) { height } else height
        var power = pid.calculate(targetHeight, encoderTicks)
        if (height > 50)
            power += .15
        left.power = power
        dashTelemetry.addData("pv", encoderTicks)
        dashTelemetry.addData("sp", height)
        dashTelemetry.update()
    }
}