package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.commands.System
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vector2
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

interface OdometrySystemState : CommandState {
    val leftOffset: Double
    val rightOffset: Double
    val backOffset: Double

    var velocity: Vector2<Double>
    var acceleration: Vector2<Double>

    var pose: Pose
}

class OdometrySystem(hardwareMap: HardwareMap) : System {
    val hub = ControlHub(hardwareMap, "Control Hub")
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")

    override val dependencies: List<System> = emptyList()
    override val beforeRun =
        Command(object : OdometrySystemState, CommandState by CommandState.default("Odometry") {
            override val leftOffset: Double = 170.2 //169.0
            override val rightOffset: Double = 170.2 //169.0
            override val backOffset: Double = 116.8 //152.4
            override var velocity: Vector2<Double> = Vector2(0.0)
            override var acceleration: Vector2<Double> = Vector2(0.0)
            override var pose: Pose = Pose(0, 0, 0)
        } as OdometrySystemState)
            .setOnEnter {
                val hub = ControlHub(hardwareMap, "Control Hub")
                hub.setJunkTicks()
            }
            .setAction {
                val odoDiameter: Double = 48.0 //MM
                val gearRatio: Double = 1.333 //1.333
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

                val deltaLeftMM = circumference * exHub.getEncoderTicks(3) / 2000
                val deltaRightMM = -circumference * exHub.getEncoderTicks(0) / 2000
                val deltaBackMM = -circumference * hub.getEncoderTicks(3) / 2000
//        println("Left: $deltaLeftMM, Right: $deltaRightMM, Back: $deltaBackMM")

                //Find the amount the robot has rotated
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

                val globalRotation = Math.toRadians(it.pose.rotation)

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
                true
            }
    override val afterRun: Command<*>? = null

    fun reset() {
        hub.setJunkTicks()
        beforeRun.state.pose = Pose(0, 0, 0)
    }
}