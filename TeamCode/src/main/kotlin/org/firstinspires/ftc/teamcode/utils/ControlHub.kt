package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit

class ControlHub(hardware: HardwareMap, hubName: String) {
    private var bulkDataCache: IntArray
    private var junkTicks = IntArray(4)

    private val hubModule: LynxModule = hardware.get(LynxModule::class.java, hubName)
    private val motorController: LynxDcMotorController = hardware.get(LynxDcMotorController::class.java, hubName)

    init {
        hubModule.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        bulkDataCache = internalRefreshBulkData()
    }

    fun getVoltage(): Double {
        return hubModule.getInputVoltage(VoltageUnit.VOLTS)
    }

    fun setJunkTicks(motor: Int? = null, junkTick: Int? = null) {
        if (motor != null)
            junkTicks[motor] = junkTick ?: bulkDataCache[motor]
        else {
            refreshBulkData()
            for (i in 0..3) {
                setJunkTicks(i)
            }
        }
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
        return bulkDataCache[motor] - junkTicks[motor]
    }
}