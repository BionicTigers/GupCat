package org.firstinspires.ftc.teamcode.utils.movement

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class Odometry(private val robot: Robot) {
    //Odometry Wheels
    private val odoDiameter: Double = 35.0 //MM
    private val gearRatio: Double = 2.5
    private val circumference: Double = odoDiameter * gearRatio * PI

    //All measurements are in MM
    private val leftOffset: Double = 162.0
    private val rightOffset: Double = 162.0
    private val backOffset: Double = 80.0

    private val hub = ControlHub(robot.hardwareMap, robot.hardwareMap.get("Control Hub") as LynxDcMotorController)

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
        val globalX = robot.pose.x + (deltaXFinal * cos(robot.pose.rotation)) + (deltaYFinal * sin(robot.pose.rotation))
        val globalY = robot.pose.y + (deltaYFinal * cos(robot.pose.rotation)) - (deltaXFinal * sin(robot.pose.rotation))

        //Update the current pose
        robot.pose = Pose(globalX, globalY, robot.pose.rotation + localRotation)

        //Reset junk ticks for next cycle
        hub.setJunkTicks()
    }

    fun reset() {
        hub.setJunkTicks()
        robot.pose = Pose()
    }
}