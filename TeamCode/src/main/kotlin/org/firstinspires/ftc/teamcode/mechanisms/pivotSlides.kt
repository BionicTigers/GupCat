package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.motion.MotionResult
import org.firstinspires.ftc.teamcode.motion.PID
import org.firstinspires.ftc.teamcode.motion.PIDTerms
import org.firstinspires.ftc.teamcode.utils.ControlHub
//import org.firstinspires.ftc.teamcode.utils.MotionResult
//import org.firstinspires.ftc.teamcode.utils.PID
//import org.firstinspires.ftc.teamcode.utils.PIDTerms
import kotlin.math.floor

/**
 * Raises and lowers arm and chainbar for more versatile scoring positions
 */
class pivotSlides(hardwareMap: HardwareMap) {

    val heightPositions = arrayOf(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0) //placeholder values

    val left = hardwareMap.get(DcMotorEx::class.java, "slideBack")
    val right = hardwareMap.get(DcMotorEx::class.java, "slideFront")
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

    var i = 0
    public fun increment() {
        i += 1
        height = heightPositions[i]
        if(i <= 0) {
            i = 0
        }
    }
    public fun decrement() {
        i -= 1
        height = heightPositions[i]
        if(i <= 0) {
            i = 0
        }
    }
    public fun reset() {
        height = heightPositions[0]
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
        if (!limitSwitch.state) {
            hub.setJunkTicks()
            height = 0.0
        }
        val encoderTicks = hub.getEncoderTicks(2).toDouble()
        val targetHeight = if (profile != null) profile!!.position.getOrElse(floor(elapsedTime.seconds() / profile!!.deltaTime).toInt()) { height } else height
        var power = pid.calculate(targetHeight, encoderTicks)
        if (height > 50)
            power += .15
        left.power = power
        right.power = power
        dashTelemetry.addData("pv", encoderTicks)
        dashTelemetry.addData("sp", height)
        dashTelemetry.update()
    }
}