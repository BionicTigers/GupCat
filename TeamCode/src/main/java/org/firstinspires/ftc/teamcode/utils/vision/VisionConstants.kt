package org.firstinspires.ftc.teamcode.utils.vision

import org.opencv.core.Scalar

object VisionConstants {
    private val ORANGE_LOWER = Scalar(6.0, 150.0, 130.0)
    private val ORANGE_UPPER = Scalar(20.0, 235.0, 185.0)

    private val GREEN_LOWER = Scalar(40.0, 100.0, 100.0)
    private val GREEN_UPPER = Scalar(74.0, 210.0, 170.0)

    private val PURPLE_LOWER = Scalar(130.0, 130.0, 21.0)
    private val PURPLE_UPPER = Scalar(180.0, 200.0, 150.0)

    val ORANGE: Color = Color(ORANGE_LOWER, ORANGE_UPPER, 1000)
    val GREEN: Color = Color(GREEN_LOWER, GREEN_UPPER, 1000)
    val PURPLE: Color = Color(PURPLE_LOWER, PURPLE_UPPER, 1000)

    const val EXPOSURE: Long = 20
    const val GAIN = 0
    const val WHITE_BALANCE = 0
}