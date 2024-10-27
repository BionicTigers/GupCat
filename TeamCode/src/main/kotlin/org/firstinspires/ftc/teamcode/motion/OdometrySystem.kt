package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.axiom.commands.*
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

interface OdometrySystemState : CommandState {
    val leftOffset: Double
    val rightOffset: Double
    val backOffset: Double

    var velocity: Vector2
    var acceleration: Vector2

    var pose: Pose
}

class OdometrySystem(hardwareMap: HardwareMap) : System {
    val hub = ControlHub(hardwareMap, "Control Hub")
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")

    override val dependencies: List<System> = emptyList()
    override val beforeRun =
        Command(object : OdometrySystemState, CommandState by CommandState.default("Odometry") {
            override val leftOffset: Double = 169.8625 //169.
            override val rightOffset: Double = 169.8625 //152.4 //169.0
            override val backOffset: Double = 95.25 //152.4 //152.4
            override var velocity: Vector2 = Vector2()
            override var acceleration: Vector2 = Vector2()
            override var pose: Pose = Pose(0, 0, 0)
        } as OdometrySystemState)
            .setOnEnter {
                hub.setJunkTicks()
                exHub.setJunkTicks()
            }
            .setAction {
                val odoDiameter = 48.0 //MM
                val gearRatio = 3.76
                val circumference: Double = odoDiameter * gearRatio * PI

                //Make new variables for local values
                val deltaLocalX: Double
                val deltaLocalY: Double

                val deltaStrafeX: Double
                val deltaStrafeY: Double

                //Find Local Updates
                hub.refreshBulkData()
                exHub.refreshBulkData()

                //Calculate how far the odo pods have moved since the last update in MM
//        val deltaLeftMM = circumference * exHub.getEncoderTicks(0) / 2000
//        val deltaRightMM = -circumference * hub.getEncoderTicks(0) / 2000
//        val deltaBackMM = circumference * hub.getEncoderTicks(3) / 2000

                val deltaLeftMM = circumference * hub.getEncoderTicks(3) / 2000
                val deltaRightMM = -circumference * exHub.getEncoderTicks(0) / 2000
                val deltaBackMM = -circumference * hub.getEncoderTicks(0) / 2000
//        println("Left: $deltaLeftMM, Right: $deltaRightMM, Back: $deltaBackMM")

                //Find the amount the robot has rotate
                val localRotation = (deltaLeftMM - deltaRightMM) / (it.leftOffset + it.rightOffset)

                //Calculates the radius of the arc of the robot's travel for forward/backward arcs
                if (deltaRightMM != deltaLeftMM && localRotation != 0.0) {
                    val rT =
                        (deltaLeftMM * it.rightOffset + deltaRightMM * it.leftOffset) / (deltaLeftMM - deltaRightMM)
                    //Determine the local x and y coordinates for a forward/backward arc
                    deltaLocalX = rT * (1 - cos(localRotation))
                    deltaLocalY = rT * sin(localRotation)
                } else {
                    deltaLocalX = 0.0
                    deltaLocalY = deltaRightMM
                }

                //Calculates the radius of a strafing arc
                if (localRotation != 0.0) {
                    val rS = (deltaBackMM / localRotation) + it.backOffset
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

                val globalRotation = it.pose.radians

                //Translate local position into global position
                val globalX =
                    it.pose.x + (deltaXFinal * cos(globalRotation)) + (deltaYFinal * sin(
                        globalRotation
                    ))
                val globalY =
                    it.pose.y + (deltaYFinal * cos(globalRotation)) - (deltaXFinal * sin(
                        globalRotation
                    ))

                //Update the current pose
                it.pose = Pose(globalX, globalY, Math.toDegrees(globalRotation + localRotation))

                //Update velocity and acceleration
                val deltaTime = it.deltaTime.seconds()
                val oldVelocity = it.velocity
                it.velocity = Vector2(deltaXFinal / deltaTime, deltaYFinal / deltaTime)
                it.acceleration = Vector2(
                    (it.velocity.x - oldVelocity.x) / deltaTime,
                    (it.velocity.y - oldVelocity.y) / deltaTime
                )

                //Reset junk ticks for next cycle
                hub.setJunkTicks()
                exHub.setJunkTicks()
                false
            }
    override val afterRun: Command<*>? = null

    val pose: Pose
        get() = beforeRun.state.pose

    fun log(telemetry: Telemetry) {
        telemetry.addData("X", beforeRun.state.pose.x)
        telemetry.addData("Y", beforeRun.state.pose.y)
        telemetry.addData("Rotation", beforeRun.state.pose.degrees)
        telemetry.addData("Velocity", beforeRun.state.velocity)
        telemetry.addData("Acceleration", beforeRun.state.acceleration)
        telemetry.update()
    }


    fun reset() {
        hub.setJunkTicks()
        exHub.setJunkTicks()
        beforeRun.state.pose = Pose(0, 0, 0)
        println("clear")
    }
}