package org.firstinspires.ftc.teamcode.utils.vision

import org.opencv.core.Scalar

class VisionConstants {
    val ORANGE_LOWER = Scalar(6.0, 150.0, 130.0)
    val ORANGE_UPPER = Scalar(20.0, 235.0, 185.0)

    val GREEN_LOWER = Scalar(40.0, 100.0, 100.0)
    val GREEN_UPPER = Scalar(74.0, 210.0, 170.0)

    val PURPLE_LOWER = Scalar(130.0, 130.0, 21.0)
    val PURPLE_UPPER = Scalar(180.0, 200.0, 150.0)

    val ORANGE: Signal = Signal(ORANGE_LOWER, ORANGE_UPPER, 1000)
    val GREEN: Signal = Signal(GREEN_LOWER, GREEN_UPPER, 1000)
    val PURPLE: Signal = Signal(PURPLE_LOWER, PURPLE_UPPER, 1000)

    val EXPOSURE = 20
    val GAIN = 0
    val WHITE_BALANCE = 0
}