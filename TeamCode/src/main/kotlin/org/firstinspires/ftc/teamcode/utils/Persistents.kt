package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.teamcode.axiom.input.Gamepad

object Persistents {
    var pose = Pose(0, 0, 0)
    var slideTicks = 0
    var pivotTicks = 0

    fun setupDriverControl(gamepad: Gamepad) {
        gamepad.getBooleanButton(Gamepad.Buttons.Y).onDown {
            reset()
        }
    }

    fun reset() {
        pose = Pose(0,0,0)
        slideTicks = 0
        pivotTicks = 0
    }
}