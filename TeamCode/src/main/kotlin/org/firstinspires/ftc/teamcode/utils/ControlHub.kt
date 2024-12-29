package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit

class ControlHub(hardware: HardwareMap, hubName: String) {
    private var bulkDataCache: IntArray
    private var junkTicks = IntArray(4)
    private var direction = IntArray(4) { 1 }

    private val hubModule: LynxModule = hardware.get(LynxModule::class.java, hubName)
    private val motorController: LynxDcMotorController = hardware.get(LynxDcMotorController::class.java, hubName)

    enum class Direction {
        Forward,
        Backward
    }

    init {
        hubModule.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        bulkDataCache = internalRefreshBulkData()
    }

    fun getVoltage(): Double {
        return hubModule.getInputVoltage(VoltageUnit.VOLTS)
    }

    fun getEncoder(port: Int): Encoder {
        assert(port in 0..3) { "Port must be between 0 and 3" }
        return Encoder(port, this)
    }

    fun setJunkTicks(motor: Int? = null, junkTick: Int? = null) {
        if (motor != null)
            junkTicks[motor] = junkTick ?: bulkDataCache[motor]
        else {
            for (i in 0..3) {
                setJunkTicks(i)
            }
        }
    }

    fun setEncoderDirection(port: Int, targetDirection: Direction) {
        direction[port] = if (targetDirection == Direction.Forward) 1 else -1
    }

    private fun internalRefreshBulkData(): IntArray {
        hubModule.clearBulkCache()
        val bulkData = IntArray(4)
        for (i in 0..3) {
            bulkData[i] = motorController.getMotorCurrentPosition(i)
        }

        return bulkData
    }

    fun refreshBulkData() {
        bulkDataCache = internalRefreshBulkData()
    }

    fun getEncoderTicks(motor: Int): Int {
        return (bulkDataCache[motor] - junkTicks[motor]) * direction[motor]
    }

    fun getAndUpdateEncoderTicks(motor: Int): Int {
        refreshBulkData()
        return getEncoderTicks(motor)
    }
}

class Encoder(private val port: Int, private val controlHub: ControlHub) {
    val ticks: Int
        get() = controlHub.getAndUpdateEncoderTicks(port)

    fun setJunkTicks(value: Int? = null) {
        println(value)
        controlHub.setJunkTicks(port, value)
    }
}