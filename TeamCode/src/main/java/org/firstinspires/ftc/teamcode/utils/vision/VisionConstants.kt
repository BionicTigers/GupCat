package org.firstinspires.ftc.teamcode.utils.vision

import com.acmerobotics.dashboard.config.Config
import org.opencv.core.Scalar

@Config
object VisionConstants {

    val RED = Color(Scalar(0.0, 120.0, 70.0), Scalar(20.0, 200.0, 180.0), 1000)
    val BLUE =  Color(Scalar(80.0, 50.0, 20.0), Scalar(140.0, 250.0, 250.0), 1000)

    const val EXPOSURE: Long = 20
    const val GAIN = 0
    const val WHITE_BALANCE = 0
}