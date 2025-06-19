package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.time.Duration
import kotlin.time.DurationUnit

inline fun <reified T : HardwareDevice> HardwareMap.getByName(name: String): T {
    return this.get(T::class.java, name)
}

val Duration.minutes: Double
    get() = this.toDouble(DurationUnit.MINUTES)
val Duration.seconds: Double
    get() = this.toDouble(DurationUnit.SECONDS)
val Duration.milliseconds: Double
    get() = this.toDouble(DurationUnit.MILLISECONDS)