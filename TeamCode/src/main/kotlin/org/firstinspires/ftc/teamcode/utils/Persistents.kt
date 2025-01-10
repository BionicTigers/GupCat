package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.teamcode.axiom.input.Gamepad

object Persistents {
    var pose = Pose(0, 0, 0)
    var slideTicks: Int? = null
    var pivotTicks: Int? = null

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.getBooleanButton(Gamepad.Buttons.Y).onDown {
            reset()
        }
    }

    fun reset() {
        println("reset")
        pose = Pose(0,0,0)
        slideTicks = null
        pivotTicks = null
    }
}