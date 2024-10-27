package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand

val dcMotorTrackers = mutableMapOf<DcMotorEx, DcMotorTracker>()

class DcMotorTracker(private val motor: DcMotorEx) {

    private var previousPosition = motor.currentPosition
    private var previousVelocity = 0.0
    var acceleration = 0.0
        private set
    var velocity = 0.0
        private set
    private var accumulativeTime = Time()

    init {
        val command = statelessCommand("DcMotor Tracker")
        .setAction {
            accumulativeTime += it.deltaTime
            if (motor.currentPosition - previousPosition == 0 || accumulativeTime <= Time.fromMilliseconds(22)) {
                return@setAction true
            }
            velocity = (motor.currentPosition - previousPosition) / accumulativeTime.seconds()
            acceleration = (velocity - previousVelocity) / accumulativeTime.seconds()
            previousPosition = motor.currentPosition
            accumulativeTime = Time()
            false
        }

        Scheduler.add(command)
    }
}

fun DcMotorEx.assignTracker() {
    if (dcMotorTrackers.containsKey(this)) {
        return
    }

    dcMotorTrackers[this] = DcMotorTracker(this)
}

fun DcMotorEx.getTracker(): DcMotorTracker {
    return dcMotorTrackers[this] ?: throw IllegalArgumentException("This motor does not have a tracker.")
}


inline fun <reified T : HardwareDevice> HardwareMap.getByName(name: String): T {
    return this.get(T::class.java, name)
}