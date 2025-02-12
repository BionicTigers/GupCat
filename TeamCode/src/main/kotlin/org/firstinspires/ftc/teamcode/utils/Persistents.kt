package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.robotcore.external.Telemetry

object Persistents {
    var pose = Pose(0, 0, 0)
    var slideTicks: Int? = null
    var pivotTicks: Int? = null

//    fun setupDriverControl(gamepad: Gamepad) {
//        gamepad.getBooleanButton(Gamepad.Buttons.Y).onDown {
//            println("y press")
//            reset()
//        }
//    }

    fun reset() {
        println("reset")
        pose = Pose(0,0,0)
        slideTicks = null
        pivotTicks = null
    }

     fun log(telemetry: Telemetry) {
         telemetry.addData("Persistents pose", pose)
         telemetry.addData("Persistents slideTicks", slideTicks)
         telemetry.addData("Persistents pivotTicks", pivotTicks)
     }
}