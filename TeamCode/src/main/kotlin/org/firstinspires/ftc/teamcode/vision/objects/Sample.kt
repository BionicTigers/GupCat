package org.firstinspires.ftc.teamcode.vision.objects

import org.firstinspires.ftc.teamcode.utils.Pose

data class Sample(val color: SampleColor, override val position: Pose) : BaseObject(position) {
    enum class SampleColor {
        Red,
        Blue,
        Yellow
    }

    object Thresholds {

    }
}