package org.firstinspires.ftc.teamcode.utils.movement

import com.qualcomm.hardware.lynx.LynxDcMotorController
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class Odometry(private val robot: Robot) {
    //Odometry Wheels
    private val odoDiameter: Double = 48.0 //MM
    private val gearRatio: Double = 1.0
    private val circumference: Double = odoDiameter * gearRatio * PI

    //All measurements are in MM
    private val leftOffset: Double = 193.7
    private val rightOffset: Double = 193.7
    private val backOffset: Double = 162.0

    //hubs
    private val hub = ControlHub(robot.hardwareMap, robot.hardwareMap.get("Control Hub") as LynxDcMotorController)
    private val exHub = ControlHub(robot.hardwareMap, robot.hardwareMap.get("Expansion Hub") as LynxDcMotorController)

    init {
        hub.setJunkTicks()
    }

    fun update() {
        //Make new variables for local values
        val deltaLocalX: Double
        val deltaLocalY: Double

        val deltaStrafeX: Double
        val deltaStrafeY: Double

        //Find Local Updates
        hub.refreshBulkData()

        //Calculate how far the odo pods have moved since the last update in MM
        val deltaLeftMM = circumference * hub.getEncoderTicks(0) / 2000
        val deltaRightMM = -circumference * hub.getEncoderTicks(3) / 2000
        val deltaBackMM = circumference * exHub.getEncoderTicks(0) / 2000
//        println("Left: $deltaLeftMM, Right: $deltaRightMM, Back: $deltaBackMM")

        //Find the amount the robot has rotated
        val localRotation = (deltaLeftMM - deltaRightMM) / (leftOffset + rightOffset)
3
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

        val globalRotation = Math.toRadians(robot.pose.rotation)

        //Translate local position into global position
        val globalX = robot.pose.x + (deltaXFinal * cos(globalRotation)) + (deltaYFinal * sin(globalRotation))
        val globalY = robot.pose.y + (deltaYFinal * cos(globalRotation)) - (deltaXFinal * sin(globalRotation))

        //Update the current pose
        robot.pose = Pose(globalX, globalY, Math.toDegrees(globalRotation + localRotation))

        //Reset junk ticks for next cycle
        hub.setJunkTicks()
    }

    fun reset() {
        hub.setJunkTicks()
        robot.pose = Pose()
    }
}