package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

class ControlHub(hardware: HardwareMap, private val hub: LynxDcMotorController) {
    private var bulkDataCache: IntArray
    private var junkTicks = IntArray(4)

    init {
        for (module in hardware.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        bulkDataCache = internalRefreshBulkData()
    }

    fun setJunkTicks() {
        refreshBulkData()
        for (i in 0..3) {
            setJunkTicks(i)
        }
    }

    private fun setJunkTicks(motor: Int) {
        junkTicks[motor] = bulkDataCache[motor]
    }

    fun setJunkTicks(motor: Int, junkTicks: Int) {
        this.junkTicks[motor] = junkTicks
    }

    private fun internalRefreshBulkData(): IntArray {
        val bulkData = IntArray(4)
        for (i in 0..3) {
            bulkData[i] = hub.getMotorCurrentPosition(i)
        }

        return bulkData
    }

    fun refreshBulkData() {
        bulkDataCache = internalRefreshBulkData()
    }

    fun getEncoderTicks(motor: Int): Int {
        return bulkDataCache[motor] - junkTicks[motor]
    }

    //TODO (Erin): toString
}