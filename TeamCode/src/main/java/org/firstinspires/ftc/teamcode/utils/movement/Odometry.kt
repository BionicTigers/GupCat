package org.firstinspires.ftc.teamcode.utils.movement

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class Odometry(hardware: HardwareMap) {
    //Odometry Wheels
    private val odoDiameter: Double = 35.0 //MM
    private val gearRatio: Double = 2.5
    private val circumference: Double = odoDiameter * gearRatio * PI

    //All measurements are in MM
    private val leftOffset: Double = 162.0
    private val rightOffset: Double = 162.0
    private val backOffset: Double = 80.0

    private val hub = ControlHub(hardware, hardware.get("Control Hub") as LynxDcMotorController)

    var globalPose: Pose = Pose()
        private set

    fun update() {
        //Make new variables for local values
        val deltaLocalX: Double
        val deltaLocalY: Double

        val deltaStrafeX: Double
        val deltaStrafeY: Double

        //Find Local Updates
        hub.refreshBulkData()

        //Calculate how far the odo pods have moved since the last update in MM
        val deltaLeftMM = circumference * hub.getEncoderTicks(0) / 8192
        val deltaRightMM = -circumference * hub.getEncoderTicks(0) / 8192
        val deltaBackMM = -circumference * hub.getEncoderTicks(0) / 8192

        //Find the amount the robot has rotated
        val localRotation = deltaLeftMM - deltaRightMM

        //Calculates the radius of the arc of the robot's travel for forward/backward arcs
        if (deltaRightMM != deltaLeftMM && localRotation != 0.0) {
            val rT = (deltaLeftMM * rightOffset + deltaRightMM * leftOffset) / (deltaLeftMM - deltaRightMM)
            //Determine the local x and y coordinates for a forward/backward arc
            deltaLocalX = rT * (1 - cos(localRotation))
            deltaLocalY = rT * sin(localRotation)
        } else {
            deltaLocalX = 0.0
            deltaLocalY = deltaRightMM
        }

        //Calculates the radius of a strafing arc
        if (localRotation != 0.0) {
            val rS = (deltaBackMM / localRotation) + backOffset
            //Determine the local x and y coordinates for a strafing arc
            deltaStrafeX = rS * sin(localRotation)
            deltaStrafeY = -rS * (1 - cos(localRotation))
        } else {
            deltaStrafeX = deltaBackMM
            deltaStrafeY = 0.0
        }


        //Calculates the total local x and y changes since last cycle
        val deltaXFinal = deltaLocalX + deltaStrafeX
        val deltaYFinal = deltaLocalY - deltaStrafeY

        //Translate local position into global position
        val globalX = globalPose.x + (deltaXFinal * cos(globalPose.rotation)) + (deltaYFinal * sin(globalPose.rotation))
        val globalY = globalPose.y + (deltaYFinal * cos(globalPose.rotation)) - (deltaXFinal * sin(globalPose.rotation))

        //Update the current pose
        globalPose = Pose(globalX, globalY, globalPose.rotation + localRotation)

        //Reset junk ticks for next cycle
        hub.setJunkTicks()
    }

    fun reset() {
        hub.setJunkTicks()
        globalPose = Pose()
    }
}