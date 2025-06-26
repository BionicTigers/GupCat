package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.HardwareMap
import com.pedropathing.localization.Localizer
import com.pedropathing.pathgen.Point
import com.pedropathing.pathgen.Vector as PathVector
import io.github.bionictigers.axiom.commands.Scheduler
import com.pedropathing.localization.Pose as PathPose
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.NewRollingAverage
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.atan2
import kotlin.time.Duration

interface RobotConfig {
    val leftOffset: Distance
    val rightOffset: Distance
    val backOffset: Distance
    val virtualOffsetX: Distance
    val virtualOffsetY: Distance
}

object Configs {
    object Test: RobotConfig {
        override val leftOffset: Distance = Distance.mm(204.0) - Distance.mm(5.0) //Distance.mm( 173.83125) + Distance.mm(10)
        override val rightOffset: Distance = Distance.mm(142.0) - Distance.mm(5.0) // Distance.mm(165.1) + Distance.mm(10) //152.4 //169.0
        override val backOffset: Distance = Distance.mm(82.0)  //Distance.mm(3/4 + 3/32) //152.4 //152.4 //95.25
        override val virtualOffsetX: Distance = Distance.mm(-68.92)  //Distance.mm(-68.97)
        override val virtualOffsetY: Distance = Distance.mm(31.0)  //Distance.mm(5.45)
    }

    object Main: RobotConfig {
        override val leftOffset: Distance = Distance.mm(171)
        override val rightOffset: Distance = Distance.mm(171)
        override val backOffset: Distance = Distance.mm(22.86)
        override val virtualOffsetX: Distance = Distance.mm(-83.82)
        override val virtualOffsetY: Distance = Distance.mm(0)
    }
}

class CustomPedroLocalizer(
    hardwareMap: HardwareMap,
    startPose: Pose = Pose(0.0, 0.0, 0.0),
    private val config: RobotConfig = Configs.Main
) : Localizer() {

    private val hub = ControlHub(hardwareMap, "Control Hub")
    private val exHub = ControlHub(hardwareMap, "Expansion Hub 2")

    private var ticksL = 0
    private var ticksR = 0
    private var ticksB = 0
    private var dt = Duration.ZERO

    // state
    private var virtualPose = computeVirtual(startPose)
    private var pose = startPose

    private var localVelocity = Vector2()
    private var localAcceleration = Vector2()
    private var globalVelocity = Pair(Vector2(), Angle.degrees(0.0))
    private var globalAcceleration = Pair(Vector2(), Angle.degrees(0.0))

    private val xAverage = NewRollingAverage(3)
    private val yAverage = NewRollingAverage(3)
    private val angularAverage = NewRollingAverage(3)

    // hardware-specific constants
    private val gearRatio = 1.0
    private val odoDiameter = 47.3
    private val ticksPerRev = 2000.0

    init {
        hub.setJunkTicks()
        exHub.setJunkTicks()
        hub.setEncoderDirection(0, ControlHub.Direction.Backward) // back pod
        hub.setEncoderDirection(3, ControlHub.Direction.Backward) // right pod
    }

    override fun update() {
        dt += Scheduler.loopDeltaTime

        val circumference = odoDiameter * gearRatio * PI

        // Find Local Updates
        hub.refreshBulkData()
        exHub.refreshBulkData()

        // Update total ticks
        ticksL += exHub.getEncoderTicks(0)
        ticksR += hub.getEncoderTicks(3)
        ticksB += hub.getEncoderTicks(0)

        // Ticks to mm
        val deltaLeft = Distance.mm(circumference * exHub.getEncoderTicks(0) / ticksPerRev)
        val deltaRight = Distance.mm(circumference * hub.getEncoderTicks(3) / ticksPerRev)
        val deltaBack = Distance.mm(circumference * hub.getEncoderTicks(0) / ticksPerRev)

        // Calculate theta
        val localRotation = Angle.radians((deltaLeft.mm - deltaRight.mm) / (config.leftOffset.mm + config.rightOffset.mm))

        // Calculate forward arc
        val rT = Distance.mm( (deltaLeft.mm / localRotation.radians) - config.leftOffset.mm)

        val deltaLocalY =
            if (localRotation.radians != 0.0)
                rT * (1 - cos(localRotation.radians))
            else
                Distance.mm(0.0)

        val deltaLocalX =
            if (localRotation.radians != 0.0)
                rT * sin(localRotation.radians)
            else
                deltaLeft

        // Calculate strafe arc
        val rS = Distance.mm(deltaBack.mm / localRotation.radians - config.backOffset.mm)

        val deltaStrafeY =
            if (localRotation.radians != 0.0)
                rS * sin(localRotation.radians)
            else
                deltaBack

        val deltaStrafeX =
            if (localRotation.radians != 0.0)
                -rS * (1 - cos(localRotation.radians))
            else
                Distance.mm(0.0)

        // Update global rotation
        val globalRotation = virtualPose.rotation

        // Calculate VIRTUAL global position
        val deltaXFinal = deltaLocalY + deltaStrafeY
        val deltaYFinal = deltaLocalX - deltaStrafeX

//        val virtualGlobalDeltaX = Distance.mm(deltaXFinal.mm * cos(globalRotation.radians) + deltaYFinal.mm * sin(globalRotation.radians))
//        val virtualGlobalDeltaY = Distance.mm(deltaYFinal.mm * cos(globalRotation.radians) - deltaXFinal.mm * sin(globalRotation.radians))

        val virtualGlobalX = Distance.mm(virtualPose.x + (deltaXFinal.mm * cos(globalRotation.radians)) + (deltaYFinal.mm * sin(globalRotation.radians)))
        val virtualGlobalY = Distance.mm(virtualPose.y + (deltaYFinal.mm * cos(globalRotation.radians)) - (deltaXFinal.mm * sin(globalRotation.radians)))

        // Update current virtual pose
        virtualPose = Pose(
            virtualGlobalX.mm,
            virtualGlobalY.mm,
            globalRotation + localRotation
        )

        // Update velocity and acceleration
        val oldVel = localVelocity
        localVelocity = Vector2((deltaLocalX + deltaStrafeX).mm / dt.seconds, (deltaLocalY + deltaStrafeY).mm / dt.seconds)

        if (abs(localVelocity.x) > abs(oldVel.x))
            localAcceleration.x = (localVelocity.x - oldVel.x) / dt.seconds

        if (abs(localVelocity.y) > abs(oldVel.y))
            localAcceleration.y = (localVelocity.y - oldVel.y) / dt.seconds

        val oldGlobalVel = globalVelocity

        xAverage += globalVelocity.first.x
        yAverage += globalVelocity.first.y
        angularAverage += globalVelocity.second.degrees

        val oldY = pose.y
        val oldX = pose.x

        // Convert virtual pose to final pose
        val y = Distance.mm(virtualPose.y - (config.virtualOffsetY * virtualPose.rotation.cos).mm + (config.virtualOffsetX * virtualPose.rotation.sin).mm)
        val x = Distance.mm(virtualPose.x - (config.virtualOffsetY * virtualPose.rotation.sin).mm - (config.virtualOffsetX * virtualPose.rotation.cos).mm)

//        val oldGlobal = Pair(Vector2(pose.x, pose.y), virtualPose.rotation)

        globalVelocity = Pair(
            Vector2((x.inch - oldX), (y.inch - oldY)) / dt.seconds,
            Angle.degrees(localRotation.degrees / dt.seconds)
        )

        globalAcceleration = Pair(
            (globalVelocity.first - oldGlobalVel.first) / dt.seconds,
            (globalVelocity.second - oldGlobalVel.second) / dt.seconds
        )

        pose = Pose(x.inch, y.inch, virtualPose.rotation)

        dt = Duration.ZERO

        hub.setJunkTicks()
        exHub.setJunkTicks()
    }

    // ---- Pedro Localizer interface ----

    override fun getPose(): PathPose =
        PathPose(pose.y, pose.x, pose.rotation.radians)

    override fun getTotalHeading(): Double =
        pose.rotation.radians

    override fun getVelocity(): PathPose =
        PathPose(globalVelocity.first.y, globalVelocity.first.x, totalHeading)

    override fun getVelocityVector(): PathVector =
        PathVector(Point(globalVelocity.first.y, globalVelocity.first.x))

    override fun isNAN(): Boolean =
        pose.x.isNaN() || pose.y.isNaN() || pose.rotation.radians.isNaN()

    override fun resetIMU() { /* no IMU to reset */ }

    override fun setPose(setPose: PathPose) {
        pose = Pose(setPose.x, setPose.y, Angle.radians(setPose.heading))

        val poseMM = Pose(
            Distance.inch(pose.y).mm,
            Distance.inch(pose.x).mm,
            pose.rotation,
        )
        virtualPose = computeVirtual(poseMM)
    }

    override fun setStartPose(start: PathPose) = setPose(start)

    override fun getForwardMultiplier(): Double = 1.0
    override fun getLateralMultiplier(): Double = 1.0
    override fun getTurningMultiplier(): Double = 1.0

    private fun computeVirtual(globalPose: Pose) = Pose(
        globalPose.y + (config.virtualOffsetX * globalPose.rotation.cos).mm - (config.virtualOffsetY * globalPose.rotation.sin).mm,
        globalPose.x + (config.virtualOffsetX * globalPose.rotation.sin).mm + (config.virtualOffsetY * globalPose.rotation.cos).mm,
        globalPose.rotation
    )
}
