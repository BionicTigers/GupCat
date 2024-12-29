package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.axiom.commands.*
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Time
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

interface OdometrySystemState : CommandState {
    val leftOffset: Distance
    val rightOffset: Distance
    val backOffset: Distance
    val virtualOffsetY: Distance
    val virtualOffsetX: Distance

    var gearRatio: Double
    var odoDiameter: Double

    var velocity: Vector2
    var acceleration: Vector2

    var virtualPose: Pose
}

class OdometrySystem(hardwareMap: HardwareMap) : System {
    val hub = ControlHub(hardwareMap, "Control Hub")
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
//    val leftOdo = hardwareMap.get(DcMotorEx::class.java, "backRight")
//    val rightOdo = hardwareMap.get(DcMotorEx::class.java, "hub3")
//    val backOdo = hardwareMap.get(DcMotorEx::class.java, "hub0")

    private var ticksL: Int = 0
    private var ticksR: Int = 0
    private var ticksB: Int = 0

    var dt: Time = Time.fromSeconds(1)

//    val limelight3A = hardwareMap.getByName<Limelight3A>("limelight")

    override val dependencies: List<System> = emptyList()
    override val beforeRun =
        Command(object : OdometrySystemState, CommandState by CommandState.default("Odometry") {
            override val leftOffset: Distance = Distance.mm(204.0) //- Distance.mm(5.0) //Distance.mm( 173.83125) + Distance.mm(10)
            override val rightOffset: Distance = Distance.mm(142.0) //- Distance.mm(5.0) // Distance.mm(165.1) + Distance.mm(10) //152.4 //169.0
            override val backOffset: Distance = Distance.mm(82.0)  //Distance.mm(3/4 + 3/32) //152.4 //152.4 //95.25
            override val virtualOffsetY: Distance = Distance.mm(-68.92)  //Distance.mm(-68.97)
            override val virtualOffsetX: Distance = Distance.mm(31.0)  //Distance.mm(5.45)
            override var velocity: Vector2 = Vector2()
            override var acceleration: Vector2 = Vector2()
            override var virtualPose = Pose(virtualOffsetX.mm, virtualOffsetY.mm, 0)
            override var odoDiameter = 47.3 //48
            override var gearRatio = 1.0
        } as OdometrySystemState)
            .setOnEnter {
                hub.setJunkTicks()
                exHub.setJunkTicks()
                it.virtualPose = Pose(it.virtualOffsetX.mm, it.virtualOffsetY.mm, 0)

                hub.setEncoderDirection(3, ControlHub.Direction.Backward) // (right pod)
            }
            .setAction {
                // Only runs odo calculations every 100 ms
                if (dt.milliseconds() < 100) {
                    dt += it.deltaTime
                    return@setAction false
                }

                val circumference: Double = it.odoDiameter * it.gearRatio * PI

                // Find Local Updates
                hub.refreshBulkData()
                exHub.refreshBulkData()

                // Update total ticks
                ticksL += exHub.getEncoderTicks(0)
                ticksR += hub.getEncoderTicks(3)
                ticksB += hub.getEncoderTicks(0)

                // TICKS TO MM
                val deltaLeftMM = Distance.mm(circumference * exHub.getEncoderTicks(0) / 2000)
                val deltaRightMM = Distance.mm(circumference * hub.getEncoderTicks(3) / 2000)
                val deltaBackMM = Distance.mm(circumference * hub.getEncoderTicks(0) / 2000)
                    //println("Left: $deltaLeftMM, Right: $deltaRightMM, Back: $deltaBackMM")

                // CALCULATE THETA
                val localRotation = Angle.radians((deltaLeftMM.mm - deltaRightMM.mm) / (it.leftOffset.mm + it.rightOffset.mm))

                // CALCULATE FORWARD ARC
                val rT = Distance.mm((deltaLeftMM.mm / localRotation.radians) - it.leftOffset.mm)

                val deltaLocalX =
                    if (localRotation.radians != 0.0) {
                        rT * (1 - cos(localRotation.radians))
                    } else {
                        Distance.mm(0.0)
                    }

                val deltaLocalY =
                    if (localRotation.radians != 0.0) {
                        rT * sin(localRotation.radians)
                    } else {
                        deltaLeftMM
                    }

                // CALCULATE STRAFE ARC
                val rS = Distance.mm(deltaBackMM.mm / localRotation.radians - it.backOffset.mm)

                val deltaStrafeX =
                    if (localRotation.radians != 0.0) {
                        rS * sin(localRotation.radians)
                    } else {
                        deltaBackMM
                    }

                val deltaStrafeY =
                    if (localRotation.radians != 0.0) {
                        -rS * (1 - cos(localRotation.radians))
                    } else {
                        Distance.mm(0.0)
                    }

                // UPDATE GLOBAL ROTATION
                val globalRotation = it.virtualPose.rotation

                // CALCULATE VIRTUAL GLOBAL POSITION
                val deltaXFinal = deltaLocalX + deltaStrafeX // final virtual delta x
                val deltaYFinal = deltaLocalY - deltaStrafeY // final virtual delta y

                val virtualGlobalX = Distance.mm(it.virtualPose.x + (deltaXFinal.mm * cos(globalRotation.radians)) + (deltaYFinal.mm * sin(globalRotation.radians)))
                val virtualGlobalY = Distance.mm(it.virtualPose.y + (deltaYFinal.mm * cos(globalRotation.radians)) - (deltaXFinal.mm * sin(globalRotation.radians)))
                //val globalY = globalVirtualY - (it.virtualOffsetY * cos(it.pose.radians)) + (it.virtualOffsetX * cos(it.pose.radians))
                //val globalX = globalVirtualX - (it.virtualOffsetY * sin(it.pose.radians)) - (it.virtualOffsetX * sin(it.pose.radians))

                // Update the current (virtual) pose
                it.virtualPose = Pose(virtualGlobalX.mm, virtualGlobalY.mm, globalRotation + localRotation) //virtual

                // Update velocity and acceleration
                val deltaTime = it.deltaTime.seconds()
                dt = it.deltaTime
                val oldVelocity = it.velocity
                it.velocity = Vector2((deltaLocalX + deltaStrafeX).mm / deltaTime, (deltaLocalY + deltaStrafeY).mm / deltaTime)
                it.acceleration = Vector2(
                    (it.velocity.x - oldVelocity.x) / deltaTime,
                    (it.velocity.y - oldVelocity.y) / deltaTime
                )

                hub.setJunkTicks()
                exHub.setJunkTicks()
                false
            }
//
//    private fun globals(x: Distance, y: Distance, rotation: Angle, svy: Distance, svx: Distance): Pair<Distance, Distance> {
//        return Pair(
//            y - (svy * rotation.cos) + (svx * rotation.sin),
//            x - (svy * rotation.sin) - (svx * rotation.cos)
//        )
//    }

    override val afterRun: Command<*>? = null

    var globalPose: Pose
        get() {
            val state = beforeRun.state
            val currentVPose = state.virtualPose

            // CONVERT VIRTUAL INTO GLOBAL
            val y = currentVPose.y - (state.virtualOffsetY * currentVPose.rotation.cos).mm + (state.virtualOffsetX * currentVPose.rotation.sin).mm
            val x = currentVPose.x - (state.virtualOffsetY * currentVPose.rotation.sin).mm - (state.virtualOffsetX * currentVPose.rotation.cos).mm

            return Pose(x, y, currentVPose.rotation)
        }
        set(value) { // TODO: make this take global poses instead of virtual
            beforeRun.state.virtualPose = value
        }

    fun log(telemetry: Telemetry) {
        telemetry.addData("XVirtual", beforeRun.state.virtualPose.x)
        telemetry.addData("X", globalPose.x)
        telemetry.addData("YVirtual", beforeRun.state.virtualPose.y)
        telemetry.addData("Y", globalPose.y)
        telemetry.addData("Rotation", globalPose.degrees)
        telemetry.addData("Velocity", beforeRun.state.velocity)
        telemetry.addData("Acceleration", beforeRun.state.acceleration)
        telemetry.addData("Ticks L", ticksL)
        telemetry.addData("Ticks R", ticksR)
        telemetry.addData("Ticks B", ticksB)
        telemetry.update()
    }


    fun reset() {
        hub.setJunkTicks()
        exHub.setJunkTicks()
        beforeRun.state.virtualPose = Pose(beforeRun.state.virtualOffsetX.mm, beforeRun.state.virtualOffsetY.mm, 0)
        println("clear")
    }
}