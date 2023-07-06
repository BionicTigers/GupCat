package org.firstinspires.ftc.teamcode.utils.vision

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

class Color(
    val lower: Scalar,
    val upper: Scalar,
    val minArea: Int,
    val maxArea: Int = Int.MAX_VALUE
) {
    lateinit var contours: ArrayList<MatOfPoint>

    fun getArea(input: Mat): Double {
        var mask: Mat = Mat()
        var temp: Mat = Mat()
        var area: Double = 0.0

        val kernel: Mat = Mat(Size(5.0, 5.0), CvType.CV_8UC1, Scalar(255.0))

        Core.inRange(input, lower, upper, mask)

        Imgproc.morphologyEx(mask, temp, Imgproc.MORPH_OPEN, kernel)

        mask.release()
        mask = Mat()
        Imgproc.morphologyEx(temp, mask, Imgproc.MORPH_CLOSE, kernel)

        contours = ArrayList()
        Imgproc.findContours(
            mask,
            contours,
            temp,
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        for (contour in contours) {
            area += Imgproc.contourArea(contour)
        }

        kernel.release()
        mask.release()
        temp.release()

        return area
    }

    fun detect(area: Double): Boolean { area > minArea && area < maxArea }
}