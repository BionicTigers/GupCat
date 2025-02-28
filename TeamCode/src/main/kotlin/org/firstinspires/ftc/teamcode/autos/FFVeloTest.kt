package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.CommandState
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import io.github.bionictigers.axiom.web.Editable
import org.firstinspires.ftc.teamcode.input.GamepadSystem
import org.firstinspires.ftc.teamcode.motion.Drivetrain
import org.firstinspires.ftc.teamcode.motion.OdometrySystem
import org.firstinspires.ftc.teamcode.utils.Pose

interface VeloTestState : CommandState {
    @Editable
    var power: Double
    @Editable
    var direction: Pose
    var odoVelocity: Pose

    companion object {
        fun default(): VeloTestState {
            return object : VeloTestState, CommandState by CommandState.default("VeloTest") {
                override var power = 1.0
                override var direction = Pose(1.0, 0.0, 0.0)
                override var odoVelocity: Pose = Pose(0, 0, 0)
            }
        }
    }
}

@Autonomous
class FFVeloTest : LinearOpMode() {
    override fun runOpMode() {
        val odometry = OdometrySystem(hardwareMap)
        val gamepad = GamepadSystem(gamepad1, gamepad2)
        val drivetrain = Drivetrain(hardwareMap, gamepad, odometry)
        drivetrain.disabled = true

        val command = Command(VeloTestState.default())
            .setAction {
                val (linear, angular) = odometry.globalVelocity
                it.odoVelocity = Pose(linear.x, linear.y, angular)
                drivetrain.veloTest(it.power, it.direction);
                false
            }

        Scheduler.addSystem(odometry, gamepad, drivetrain)
        Scheduler.add(command)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
        }
    }
}