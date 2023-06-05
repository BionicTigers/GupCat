package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.Robot

class lift(hardwareMap: HardwareMap, private val robot: Robot) {
    private var top = hardwareMap.get(DcMotorEx::class.java, "top")
    private var middle = hardwareMap.get(DcMotorEx::class.java, "middle")
    private var bottom = hardwareMap.get(DcMotorEx::class.java, "bottom")
    private var limitSwitch = hardwareMap.get(DigitalChannel::class.java, "limitSwitch")

    public var targetHeight: Int = 0
    public var trim: Int = 0

    public var bumpedUp: Boolean = false
    public var bumpedDown: Boolean = false
    public var killPower: Boolean = true
    public var activelyDown: Boolean = false

    fun determineHeightLift (tarHight: Int, kPower: Boolean){
        targetHeight = tarHight
        killPower = kPower
    }
    fun notTrim (ten: Int){
        trim += ten
    }


}