package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.defaultCommand

val dcMotorTrackers = mutableMapOf<DcMotorEx, DcMotorTracker>()

class DcMotorTracker(private val motor: DcMotorEx) {
    private var deltaTime = Time()
    val acceleration
        get() = (velocity - previousVelocity) / deltaTime.seconds()
    private var previousVelocity = 0.0
    var velocity = motor.velocity

    init {
        val command = defaultCommand("DcMotor Tracker")
        .setAction {
                previousVelocity = velocity
                velocity = motor.velocity
                deltaTime = it.deltaTime
                true
            }
        Scheduler.add(command)
    }
}

fun DcMotorEx.assignTracker(setup: Boolean = true) {
    if (dcMotorTrackers.containsKey(this)) {
        return
    }

    dcMotorTrackers[this] = DcMotorTracker(this)

    if (setup) this.mode = DcMotor.RunMode.RUN_USING_ENCODER
}

fun DcMotorEx.getTracker(): DcMotorTracker {
    return dcMotorTrackers[this] ?: throw IllegalArgumentException("This motor does not have a tracker.")
}
