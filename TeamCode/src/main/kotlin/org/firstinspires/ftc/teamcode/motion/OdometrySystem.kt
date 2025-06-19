package org.firstinspires.ftc.teamcode.localization

import com.qualcomm.robotcore.hardware.HardwareMap
import com.pedropathing.localization.Localizer
import com.pedropathing.pathgen.Vector as PathVector
import io.github.bionictigers.axiom.commands.Scheduler
import com.pedropathing.localization.Pose as PathPose
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.Angle
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.NewRollingAverage

interface RobotConfig {
    val leftOffset: Distance
    val rightOffset: Distance
    val backOffset: Distance
    val virtualOffsetY: Distance
    val virtualOffsetX: Distance
}

private object Configs {
    object Test: RobotConfig {
        override val leftOffset: Distance = Distance.mm(204.0) - Distance.mm(5.0) //Distance.mm( 173.83125) + Distance.mm(10)
        override val rightOffset: Distance = Distance.mm(142.0) - Distance.mm(5.0) // Distance.mm(165.1) + Distance.mm(10) //152.4 //169.0
        override val backOffset: Distance = Distance.mm(82.0)  //Distance.mm(3/4 + 3/32) //152.4 //152.4 //95.25
        override val virtualOffsetY: Distance = Distance.mm(-68.92)  //Distance.mm(-68.97)
        override val virtualOffsetX: Distance = Distance.mm(31.0)  //Distance.mm(5.45)
    }

    object Main: RobotConfig {
        override val leftOffset: Distance = Distance.mm(171)
        override val rightOffset: Distance = Distance.mm(171)
        override val backOffset: Distance = Distance.mm(22.86)
        override val virtualOffsetY: Distance = Distance.mm(-83.82)
        override val virtualOffsetX: Distance = Distance.mm(0)
    }
}
/**
 * A Pedro-Pathing Localizer that internally uses your OdometrySystem logic.
 */
class CustomPedroLocalizer(
    hardwareMap: HardwareMap,
    startPose: Pose = Pose(0.0, 0.0, 0.0),
    private val config: RobotConfig = Configs.Main
) : Localizer() {
    // re-use your existing hubs & config
    private val hub = org.firstinspires.ftc.teamcode.utils.ControlHub(hardwareMap, "Control Hub")
    private val exHub = org.firstinspires.ftc.teamcode.utils.ControlHub(hardwareMap, "Expansion Hub 2")

    private val ticksPerRev = 2000.0
    private var ticksL = 0
    private var ticksR = 0
    private var ticksB = 0
    private var dt = Time.fromSeconds(1.0)

    // state mirrors your OdometrySystemState
    private var virtualPose = computeVirtual(startPose)
    private var pose = startPose

    private var localVelocity = Vector2()
    private var localAcceleration = Vector2()
    private var globalVelocity = Pair(Vector2(), Angle.degrees(0.0))
    private var globalAcceleration = Pair(Vector2(), Angle.degrees(0.0))

    private val xAvg = NewRollingAverage(3)
    private val yAvg = NewRollingAverage(3)
    private val angAvg = NewRollingAverage(3)

    // hardware-specific constants
    private val gearRatio = 1.0
    private val odoDiameter = 47.3

    init {
        // exactly the same startup you had
        hub.setJunkTicks()
        exHub.setJunkTicks()
        hub.setEncoderDirection(0, org.firstinspires.ftc.teamcode.utils.ControlHub.Direction.Backward)
        hub.setEncoderDirection(3, org.firstinspires.ftc.teamcode.utils.ControlHub.Direction.Backward)
    }

    /** Copy of your OdometrySystem setAction body, run once per loop. */
    override fun update() {
        dt += Scheduler.loopDeltaTime

        val circumference = odoDiameter * gearRatio * PI

        hub.refreshBulkData();  exHub.refreshBulkData()

        // accumulate total ticks
        ticksL += exHub.getEncoderTicks(0)
        ticksR += hub.getEncoderTicks(3)
        ticksB += hub.getEncoderTicks(0)

        // convert to mm
        val dL = Distance.mm(circumference * exHub.getEncoderTicks(0) / ticksPerRev)
        val dR = Distance.mm(circumference * hub.getEncoderTicks(3) / ticksPerRev)
        val dB = Distance.mm(circumference * hub.getEncoderTicks(0) / ticksPerRev)

        // three-wheel odometry
        val dTheta = Angle.radians((dL.mm - dR.mm) / (config.leftOffset.mm + config.rightOffset.mm))

        val rT = Distance.mm(dL.mm / dTheta.radians - config.leftOffset.mm)
        val dxL = if (dTheta.radians != 0.0) rT * (1 - cos(dTheta.radians)) else Distance.mm(0.0)
        val dyL = if (dTheta.radians != 0.0) rT * sin(dTheta.radians) else dL

        val rS = Distance.mm(dB.mm / dTheta.radians - config.backOffset.mm)
        val dxS = if (dTheta.radians != 0.0) rS * sin(dTheta.radians) else dB
        val dyS = if (dTheta.radians != 0.0) -rS * (1 - cos(dTheta.radians)) else Distance.mm(0.0)

        // world update
        val worldHeading = virtualPose.rotation
        val deltaX = dxL + dxS
        val deltaY = dyL - dyS

        val globDX = Distance.mm(deltaX.mm * cos(worldHeading.radians) + deltaY.mm * sin(worldHeading.radians))
        val globDY = Distance.mm(deltaY.mm * cos(worldHeading.radians) - deltaX.mm * sin(worldHeading.radians))

        virtualPose = Pose(
            virtualPose.x + globDX.mm,
            virtualPose.y + globDY.mm,
            worldHeading + dTheta
        )

        // compute velocity / accel
        val oldVel = localVelocity
        localVelocity = Vector2((deltaX.mm + deltaY.mm) / dt.seconds(), (deltaY.mm + deltaY.mm) / dt.seconds())
        if (abs(localVelocity.x) > abs(oldVel.x))
            localAcceleration.x = (localVelocity.x - oldVel.x) / dt.seconds()
        if (abs(localVelocity.y) > abs(oldVel.y))
            localAcceleration.y = (localVelocity.y - oldVel.y) / dt.seconds()

        xAvg += globalVelocity.first.x
        yAvg += globalVelocity.first.y
        angAvg += globalVelocity.second.degrees

        // final pose in robot frame
        val finalX = virtualPose.x - (config.virtualOffsetY * virtualPose.rotation.cos).mm + (config.virtualOffsetX * virtualPose.rotation.sin).mm
        val finalY = virtualPose.y - (config.virtualOffsetY * virtualPose.rotation.sin).mm - (config.virtualOffsetX * virtualPose.rotation.cos).mm

        val oldGlobal = Pair(Vector2(pose.x, pose.y), virtualPose.rotation)
        pose = Pose(finalX, finalY, virtualPose.rotation)

        globalVelocity = Pair(
            Vector2((pose.x - oldGlobal.first.x) / dt.seconds(), (pose.y - oldGlobal.first.y) / dt.seconds()),
            Angle.degrees(dTheta.degrees / dt.seconds())
        )
        globalAcceleration = Pair(
            (globalVelocity.first  - oldGlobal.first) / dt.seconds(),
            (globalVelocity.second - oldGlobal.second) / dt.seconds()
        )

        dt = Time.fromSeconds(0.0)
    }

    // ---- Pedro Localizer interface ----

    override fun getPose(): PathPose =
        PathPose(pose.x, pose.y, pose.rotation.radians)

    override fun getTotalHeading(): Double =
        pose.rotation.radians

    override fun getVelocity(): PathPose =
        PathPose(globalVelocity.first.x, globalVelocity.first.y, globalVelocity.second.radians)

    override fun getVelocityVector(): PathVector =
        PathVector(globalVelocity.first.x, globalVelocity.first.y)

    override fun isNAN(): Boolean =
        pose.x.isNaN() || pose.y.isNaN() || pose.rotation.radians.isNaN()

    override fun resetIMU() { /* no IMU to reset */ }

    override fun setPose(setPose: PathPose) {
        pose = Pose(setPose.x, setPose.y, Angle.radians(setPose.heading))
        virtualPose = computeVirtual(pose)
    }

    override fun setStartPose(start: PathPose) = setPose(start)

    override fun getForwardMultiplier(): Double = 1.0
    override fun getLateralMultiplier(): Double = 1.0
    override fun getTurningMultiplier(): Double = 1.0

    private fun computeVirtual(p: Pose) = Pose(
        p.y + (config.virtualOffsetY * p.rotation.cos).mm - (config.virtualOffsetX * p.rotation.sin).mm,
        p.x + (config.virtualOffsetY * p.rotation.sin).mm + (config.virtualOffsetX * p.rotation.cos).mm,
        p.rotation
    )
}
