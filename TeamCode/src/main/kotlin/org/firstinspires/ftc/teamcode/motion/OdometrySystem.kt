package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.hardware.bosch.BNO055IMUNew
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vector2
import io.github.bionictigers.axiom.commands.*
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Persistents
import io.github.bionictigers.axiom.utils.Time
import io.github.bionictigers.axiom.web.WebData
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

interface OdometrySystemState : CommandState {
    val leftOffset: Distance
    val rightOffset: Distance
    val backOffset: Distance
    val virtualOffsetY: Distance
    val virtualOffsetX: Distance

    var gearRatio: Double
    var odoDiameter: Double

    var localVelocity: Vector2
    var localAcceleration: Vector2
    var globalVelocity: Pair<Vector2, Angle>
    var globalAcceleration: Pair<Vector2, Angle>

    var virtualPose: Pose
}

class OdometrySystem(hardwareMap: HardwareMap, initialPose: Pose? = null) : System {
    val hub = ControlHub(hardwareMap, "Control Hub")
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")
//    val leftOdo = hardwareMap.get(DcMotorEx::class.java, "backRight")
//    val rightOdo = hardwareMap.get(DcMotorEx::class.java, "hub3")
//    val backOdo = hardwareMap.get(DcMotorEx::class.java, "hub0")

    private var ticksL: Int = 0
    private var ticksR: Int = 0
    private var ticksB: Int = 0

    var dt: Time = Time.fromSeconds(1)

    companion object {
        object Test {
            val leftOffset: Distance = Distance.mm(204.0) - Distance.mm(5.0) //Distance.mm( 173.83125) + Distance.mm(10)
            val rightOffset: Distance = Distance.mm(142.0) - Distance.mm(5.0) // Distance.mm(165.1) + Distance.mm(10) //152.4 //169.0
            val backOffset: Distance = Distance.mm(82.0)  //Distance.mm(3/4 + 3/32) //152.4 //152.4 //95.25
            val virtualOffsetY: Distance = Distance.mm(-68.92)  //Distance.mm(-68.97)
            val virtualOffsetX: Distance = Distance.mm(31.0)  //Distance.mm(5.45)
        }

        object Main {
            val leftOffset: Distance = Distance.mm(171)
            val rightOffset: Distance = Distance.mm(171)
            val backOffset: Distance = Distance.mm(70.8)
            val virtualOffsetY: Distance = Distance.mm(0)
            val virtualOffsetX: Distance = Distance.mm(23)
        }
    }

    //    val limelight3A = hardwareMap.getByName<Limelight3A>("limelight")
    val robotConfig = Main

    override val dependencies: List<System> = emptyList()
    override val beforeRun =
        Command(object : OdometrySystemState, CommandState by CommandState.default("Odometry") {
            override val leftOffset: Distance = robotConfig.leftOffset
            override val rightOffset: Distance = robotConfig.rightOffset
            override val backOffset: Distance = robotConfig.backOffset
            override val virtualOffsetY: Distance = robotConfig.virtualOffsetY
            override val virtualOffsetX: Distance = robotConfig.virtualOffsetX
            override var localVelocity: Vector2 = Vector2()
            override var localAcceleration: Vector2 = Vector2()
            override var globalVelocity: Pair<Vector2, Angle> = Pair(Vector2(), Angle.degrees(0.0))
            override var globalAcceleration: Pair<Vector2, Angle> = Pair(Vector2(), Angle.degrees(0.0))
            override var virtualPose = Pose(virtualOffsetX.mm, virtualOffsetY.mm, 0)
            override var odoDiameter = 47.3 //48
            override var gearRatio = 1.0
        } as OdometrySystemState)
            .setOnEnter {
                hub.setJunkTicks()
                exHub.setJunkTicks()

                hub.setEncoderDirection(0, ControlHub.Direction.Backward) // (back pod)
                hub.setEncoderDirection(3, ControlHub.Direction.Backward) // (right pod)

                it.virtualPose = initialPose ?: Persistents.pose
            }
            .setAction {
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

                val virtualGlobalDeltaX = (deltaXFinal.mm * cos(globalRotation.radians)) + (deltaYFinal.mm * sin(globalRotation.radians))
                val virtualGlobalDeltaY = (deltaYFinal.mm * cos(globalRotation.radians)) - (deltaXFinal.mm * sin(globalRotation.radians))

                val virtualGlobalX = Distance.mm(it.virtualPose.x + (deltaXFinal.mm * cos(globalRotation.radians)) + (deltaYFinal.mm * sin(globalRotation.radians)))
                val virtualGlobalY = Distance.mm(it.virtualPose.y + (deltaYFinal.mm * cos(globalRotation.radians)) - (deltaXFinal.mm * sin(globalRotation.radians)))
                //val globalY = globalVirtualY - (it.virtualOffsetY * cos(it.pose.radians)) + (it.virtualOffsetX * cos(it.pose.radians))
                //val globalX = globalVirtualX - (it.virtualOffsetY * sin(it.pose.radians)) - (it.virtualOffsetX * sin(it.pose.radians))

                // Update the current (virtual) pose
                it.virtualPose = Pose(virtualGlobalX.mm, virtualGlobalY.mm, globalRotation + localRotation) //virtual
                Persistents.pose = it.virtualPose

                // Update velocity and acceleration
                val deltaTime = it.deltaTime.seconds()
                dt = it.deltaTime

                val oldVelocity = it.localVelocity
                it.localVelocity = Vector2((deltaLocalX + deltaStrafeX).mm / deltaTime, (deltaLocalY + deltaStrafeY).mm / deltaTime)
                it.localAcceleration = Vector2((it.localVelocity.x - oldVelocity.x) / deltaTime, (it.localVelocity.y - oldVelocity.y) / deltaTime)

                val oldGlobalVel = it.globalVelocity
                it.globalVelocity = Pair(Vector2(virtualGlobalDeltaX, virtualGlobalDeltaY) / deltaTime, Angle.degrees(localRotation.degrees / deltaTime))
                it.globalAcceleration = Pair(oldGlobalVel.first - it.globalVelocity.first, oldGlobalVel.second - it.globalVelocity.second)

                // CONVERT VIRTUAL INTO GLOBAL
                val y = it.virtualPose.y - (it.virtualOffsetY * it.virtualPose.rotation.cos).mm + (it.virtualOffsetX * it.virtualPose.rotation.sin).mm
                val x = it.virtualPose.x - (it.virtualOffsetY * it.virtualPose.rotation.sin).mm - (it.virtualOffsetX * it.virtualPose.rotation.cos).mm

                _globalPose = Pose(x, y, it.virtualPose.rotation)

                // Update to web data
                WebData.x = x
                WebData.y = y
                WebData.rotation = it.virtualPose.rotation.radians

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

    private var _globalPose: Pose = Pose(0, 0, 0)
    var globalPose: Pose
        get() = _globalPose
        set(value) {
            val state = beforeRun.state

            val virtualY = value.y + (state.virtualOffsetY * value.rotation.cos).mm - (state.virtualOffsetX * value.rotation.sin).mm
            val virtualX = value.x + (state.virtualOffsetY * value.rotation.sin).mm + (state.virtualOffsetX * value.rotation.cos).mm
            beforeRun.state.virtualPose = Pose(virtualX, virtualY, value.rotation)
        }

    val globalVelocity: Pair<Vector2, Angle>
        get() = beforeRun.state.globalVelocity


    var localMaxAccelX = 0.0
    var localMaxAccelY = 0.0
    var localMaxVelocityX = 0.0
    var localMaxVelocityY = 0.0

    var angularMaxAccel = Angle.degrees(0.0)
    var angularMaxVelocity = Angle.degrees(0.0)

    fun log(telemetry: Telemetry) {
        val (linearVelocity, angularVelocity) = beforeRun.state.globalVelocity
        localMaxAccelX = max(localMaxAccelX, beforeRun.state.localAcceleration.x)
        localMaxAccelY = max(localMaxAccelY, beforeRun.state.localAcceleration.y)
        localMaxVelocityX = max(localMaxVelocityX, beforeRun.state.localVelocity.x)
        localMaxVelocityY = max(localMaxVelocityY, beforeRun.state.localVelocity.y)
//        angularMaxAccel = Angle.degrees(max(angularMaxAccel.degrees, abs(beforeRun.state.globalAcceleration.second.degrees)))
//        angularMaxVelocity = Angle.degrees(max(angularMaxVelocity.degrees, abs(beforeRun.state.globalVelocity.second.degrees)))

        telemetry.addData("XVirtual", beforeRun.state.virtualPose.x)
        telemetry.addData("X", globalPose.x)
        telemetry.addData("YVirtual", beforeRun.state.virtualPose.y)
        telemetry.addData("Y", globalPose.y)
        telemetry.addData("Rotation", globalPose.degrees)

        telemetry.addData("Velocity", beforeRun.state.localVelocity)
        telemetry.addData("Acceleration", beforeRun.state.localAcceleration)
        telemetry.addData("maxAccel", "($localMaxAccelX, $localMaxAccelY)")
        telemetry.addData("maxVelocity", "(${localMaxVelocityX}, $localMaxVelocityY)")
//        telemetry.addData("angularVelocity", angularMaxVelocity.degrees)
//        telemetry.addData("angularAcceleration", angularMaxAccel.degrees)

        telemetry.addData("Ticks L", ticksL)
        telemetry.addData("Ticks R", ticksR)
        telemetry.addData("Ticks B", ticksB)
    }

    @Suppress("unused")
    fun logPosition(telemetry: Telemetry) {
        telemetry.addData("XVirtual", beforeRun.state.virtualPose.x)
        telemetry.addData("X", globalPose.x)
        telemetry.addData("YVirtual", beforeRun.state.virtualPose.y)
        telemetry.addData("Y", globalPose.y)
        telemetry.addData("Rotation", globalPose.degrees)
    }

    @Suppress("unused")
    fun logMaximums(telemetry: Telemetry) {
        localMaxAccelX = max(localMaxAccelX, beforeRun.state.localAcceleration.x)
        localMaxAccelY = max(localMaxAccelY, beforeRun.state.localAcceleration.y)
        localMaxVelocityX = max(localMaxVelocityX, beforeRun.state.localVelocity.x)
        localMaxVelocityY = max(localMaxVelocityY, beforeRun.state.localVelocity.y)
        angularMaxAccel = Angle.degrees(max(angularMaxAccel.degrees, abs(beforeRun.state.globalAcceleration.second.degrees)))
        angularMaxVelocity = Angle.degrees(max(angularMaxVelocity.degrees, abs(beforeRun.state.globalVelocity.second.degrees)))

        telemetry.addData("Velocity", beforeRun.state.localVelocity)
        telemetry.addData("Acceleration", beforeRun.state.localAcceleration)
        telemetry.addData("maxAccel", "($localMaxAccelX, $localMaxAccelY)")
        telemetry.addData("maxVelocity", "(${localMaxVelocityX}, $localMaxVelocityY)")
        telemetry.addData("angularVelocity", angularMaxVelocity.degrees)
        telemetry.addData("angularAcceleration", angularMaxAccel.degrees)
    }

    @Suppress("unused")
    fun logTicks(telemetry: Telemetry) {
        telemetry.addData("Ticks L", ticksL)
        telemetry.addData("Ticks R", ticksR)
        telemetry.addData("Ticks B", ticksB)
    }

    fun reset() {
        hub.setJunkTicks()
        exHub.setJunkTicks()
        beforeRun.state.virtualPose = Pose(beforeRun.state.virtualOffsetX.mm, beforeRun.state.virtualOffsetY.mm, 0)
        println("clear")
    }
}