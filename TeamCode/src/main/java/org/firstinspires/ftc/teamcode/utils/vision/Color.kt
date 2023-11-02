package org.firstinspires.ftc.teamcode.utils.vision

import org.firstinspires.ftc.teamcode.utils.Vector2
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

    fun calculate(input: Mat): Pair<Double, Vector2> {
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

        var largest: Pair<Double, MatOfPoint> = Pair(0.0, MatOfPoint())
        for (contour in contours) {
            val size = Imgproc.contourArea(contour)
            area += size
            if (size > largest.first) {
                largest = Pair(size, contour)
            }
        }

        kernel.release()
        mask.release()
        temp.release()

        //Calculate center point of contour
        val moments = Imgproc.moments(largest.second)
        return Pair(area, Vector2(moments.m10 / moments.m00, moments.m01 / moments.m00))
    }

    fun detect(area: Double): Boolean {return area > minArea && area < maxArea }
}