package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Drone
import org.firstinspires.ftc.teamcode.utils.Range
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.command.Scheduler
import org.firstinspires.ftc.teamcode.utils.command.continuousCommand
import org.firstinspires.ftc.teamcode.utils.input.GamepadEx
import kotlin.math.cos
import kotlin.math.sin

@TeleOp
class FindYourStripes : LinearOpMode() {
    override fun runOpMode() {
        val field = Range(-1900.0/2, 1900.0/2)

        val robot = Robot(this)
        val drivetrain = Drivetrain(hardwareMap, robot)
        val drone = Drone(hardwareMap)

        val (gp1, gp2) = robot.getGamepads()
        val (leftJoystick, rightJoystick) = gp1.getJoysticks()

        var override = false

        gp1.getButton(GamepadEx.Buttons.B).onStart {
            override = true
        }

        gp1.getButton(GamepadEx.Buttons.B).onEnd {
            override = false
        }

        Scheduler.add(continuousCommand {
//            if ((field.within(robot.pose.x) && field.within(robot.pose.y)) || override) {
                drivetrain.robotDMP(leftJoystick.state, -rightJoystick.state.x)
//            } else {
//                drivetrain.stop()
//            }
        })

        gp1.getButton(GamepadEx.Buttons.X).onStart {
            drone.start()
        }

        robot.onStart {
            Scheduler.update()
        }

        Scheduler.clear()
    }
}