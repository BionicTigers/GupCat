package org.firstinspires.ftc.teamcode.motion

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Rotation
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

    var gearRatio: Double
    var odoDiameter: Double

    var velocity: Vector2
    var acceleration: Vector2

    var pose: Pose
}

class OdometrySystem(hardwareMap: HardwareMap) : System {
    val hub = ControlHub(hardwareMap, "Control Hub")
    val exHub = ControlHub(hardwareMap, "Expansion Hub 2")

    var dt: Time = Time.fromSeconds(1)

//    val limelight3A = hardwareMap.getByName<Limelight3A>("limelight")

    override val dependencies: List<System> = emptyList()
    override val beforeRun =
        Command(object : OdometrySystemState, CommandState by CommandState.default("Odometry") {
            override val leftOffset: Distance = Distance.mm(167.878)
            override val rightOffset: Distance = Distance.mm(167.878) //152.4 //169.0
            override val backOffset: Distance = Distance.mm(95.25) //152.4 //152.4
            override var velocity: Vector2 = Vector2()
            override var acceleration: Vector2 = Vector2()
            override var pose: Pose = Pose(0, 0, 0)
            override var odoDiameter = 48.0
            override var gearRatio = 1.0
        } as OdometrySystemState)
            .setOnEnter {
                hub.setJunkTicks()
                exHub.setJunkTicks()
            }
            .setAction {
                if (dt.milliseconds() < 100) {
                    dt += it.deltaTime
                    return@setAction false
                }

                val circumference: Double = it.odoDiameter * it.gearRatio * PI

                //Find Local Updates
                hub.refreshBulkData()
                exHub.refreshBulkData()

                val (deltaLeft, deltaRight, deltaBack) = ticksToDistance(circumference, 2000)

                val localRotation = deltaTheta(deltaLeft, deltaRight, it.leftOffset, it.rightOffset)

                val (deltaLocalX, deltaLocalY) = forwardArc(deltaLeft, it.leftOffset, localRotation)
                val (deltaStrafeX, deltaStrafeY) = strafeArc(deltaBack, it.backOffset, localRotation)

                val globalRotation = it.pose.rotation
                val (globalX, globalY) = globals(it.pose, deltaLocalX, deltaLocalY, deltaStrafeX, deltaStrafeY, globalRotation)
                it.pose = Pose(globalX.mm, globalY.mm, globalRotation + localRotation)

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

    private fun deltaTheta(deltaLeft: Distance, deltaRight: Distance, leftOffset: Distance, rightOffset: Distance): Angle = Angle.radians((deltaLeft.mm - deltaRight.mm) / (leftOffset.mm + rightOffset.mm))

    @Suppress("SameParameterValue")
    private fun ticksToDistance(circumference: Double, ticksPerRevolution: Int): Triple<Distance, Distance, Distance> {
        val deltaLeftMM = Distance.mm(-circumference * exHub.getEncoderTicks(0) / ticksPerRevolution)
        val deltaRightMM = Distance.mm(circumference * hub.getEncoderTicks(3) / ticksPerRevolution)
        val deltaBackMM = Distance.mm(-circumference * hub.getEncoderTicks(0) / ticksPerRevolution)

        return Triple(deltaLeftMM, deltaRightMM, deltaBackMM)
    }

    private fun forwardArc(deltaLeft: Distance, leftOffset: Distance, localRotation: Angle): Pair<Distance, Distance> {
        if (localRotation.radians != 0.0) {
            val rT = Distance.mm(deltaLeft.mm / localRotation.radians - leftOffset.mm)
            return Pair(
                rT * (1 - cos(localRotation.radians)), // X
                rT * sin(localRotation.radians) // Y
            )
        }

        return Pair(
            Distance.mm(0.0), // X
            deltaLeft // Y
        )
    }

    private fun strafeArc(deltaBack: Distance, backOffset: Distance, localRotation: Angle): Pair<Distance, Distance> {
        if (localRotation.radians != 0.0) {
            val rS = Distance.mm(deltaBack.mm / localRotation.radians - backOffset.mm)
            return Pair(
                rS * sin(localRotation.radians),
                -rS * (1 - cos(localRotation.radians))
            )
        }

        return Pair(
            deltaBack,
            Distance.mm(0.0)
        )
    }

    private fun globals(pose: Pose, deltaLocalX: Distance, deltaLocalY: Distance, deltaStrafeX: Distance, deltaStrafeY: Distance, rotation: Angle): Pair<Distance, Distance> {
        val deltaXFinal = deltaLocalX + deltaStrafeX
        val deltaYFinal = deltaLocalY - deltaStrafeY

        return Pair(
            Distance.mm(pose.x + (deltaXFinal.mm * cos(rotation.radians)) + (deltaYFinal.mm * sin(rotation.radians))),
            Distance.mm(pose.y + (deltaYFinal.mm * cos(rotation.radians)) - (deltaXFinal.mm * sin(rotation.radians)))
        )
    }

    override val afterRun: Command<*>? = null

    var pose: Pose
        get() = beforeRun.state.pose
        set(value) {
            beforeRun.state.pose = value
        }

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