package org.firstinspires.ftc.teamcode.utils.vision

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

class Signal {
    var lower: Scalar? = null
    var upper: Scalar? = null

    var contours: ArrayList<MatOfPoint>? = null

    var minArea = 0.0
    var maxArea = 0.0

    //Create a new signal with the upper and lower bounds and a minimum area with no maximum
    //The Scalar for lower and upper goes (H, S, V)
    fun Signal(lower: Scalar?, upper: Scalar?, minArea: Int) {
        //Set the lower and upper bounds for HSV Values
        this.lower = lower
        this.upper = upper

        //Set the minimum area
        //Sets the maximum area to infinity
        this.minArea = minArea.toDouble()
        maxArea = Int.MAX_VALUE.toDouble()
    }

    //Create a new signal with the upper and lower bounds and a minimum and maximum area
    //The Scalar for lower and upper goes (H, S, V)
    fun Signal(lower: Scalar?, upper: Scalar?, minArea: Int, maxArea: Int) {
        //Set the lower and upper bounds for HSV Values
        this.lower = lower
        this.upper = upper

        //Set the minimum area and maximum area
        this.minArea = minArea.toDouble()
        this.maxArea = maxArea.toDouble()
    }

    fun getArea(input: Mat?): Double {
        //Create starting variables
        var mask = Mat()
        val temp = Mat()
        var area = 0.0

        //Create a kernel for morphological operations
        val kernel = Mat(Size(5.0, 5.0), CvType.CV_8UC1, Scalar(255.0))

        //Create a color mask
        Core.inRange(input, lower, upper, mask)

        //Preform morphological operation to remove noise
        Imgproc.morphologyEx(mask, temp, Imgproc.MORPH_OPEN, kernel)

        //Release mask to be about to use at the output then reassign it
        mask.release()
        mask = Mat()
        Imgproc.morphologyEx(temp, mask, Imgproc.MORPH_CLOSE, kernel)

        //Create a new ArrayList of contours and then find said contours to map
        contours = ArrayList()
        Imgproc.findContours(
            mask,
            contours,
            temp,
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        //Sum the area of every contour
        for (contour in contours!!) {
            area += Imgproc.contourArea(contour)
        }

        //Free the mat's from memory to prevent a memory leak
        kernel.release()
        mask.release()
        temp.release()
        return area
    }

    //Used to check if the area falls inside of maxArea
    fun detect(area: Double): Boolean {
        return area > minArea && area < maxArea
    }

}