package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.commands.Scheduler
import io.github.bionictigers.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.mechanisms.Pivot
import org.firstinspires.ftc.teamcode.mechanisms.Slides
import kotlin.math.abs


@Autonomous
class PivotTest : LinearOpMode(){
    override fun runOpMode() {
        val slides = Slides(hardwareMap)
        val pivot = Pivot(hardwareMap, slides)
        slides.pivot = pivot

        Scheduler.add(statelessCommand("move")
            .setOnEnter { slides.mpMove(slides.max) }
            .setAction { return@setAction abs(slides.ticks - slides.max) < 400 }
            .setOnExit { slides.mpMove(0) }
        )

        Scheduler.addSystem(slides, pivot)

        waitForStart()
        while (opModeIsActive()) {
            Scheduler.update()
        }
    }
}