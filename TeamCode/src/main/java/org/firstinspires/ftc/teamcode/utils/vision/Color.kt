package org.firstinspires.ftc.teamcode.utils.vision

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

class Color(
        private val lower: Scalar,
        private val upper: Scalar,
        private val minArea: Int,
        private val maxArea: Int = Int.MAX_VALUE
) {
    lateinit var contours: ArrayList<MatOfPoint>

    fun getArea(input: Mat): Double {
        var mask = Mat()
        val temp = Mat()
        var area = 0.0

        val kernel = Mat(Size(5.0, 5.0), CvType.CV_8UC1, Scalar(255.0))

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

    fun detect(area: Double): Boolean {return area > minArea && area < maxArea }
}