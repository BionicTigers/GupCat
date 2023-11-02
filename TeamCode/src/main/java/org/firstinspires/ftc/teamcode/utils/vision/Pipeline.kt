package org.firstinspires.ftc.teamcode.utils.vision

import org.firstinspires.ftc.teamcode.utils.Vector2
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

data class Detection(val name: String, val position: Vector2)

class Pipeline(private val injection: OpenCv) : OpenCvPipeline() {

    private var detection: Detection? = null

    //This is essentially the Main method for the pipeline
    //On every new frame from the camera, this is ran
    override fun processFrame(input: Mat): Mat? {
        detection = null

        val hsvMatUncropped = Mat()

        val contours = ArrayList<MatOfPoint>()

        //Convert the input image into HSV
        Imgproc.cvtColor(input, hsvMatUncropped, Imgproc.COLOR_RGB2HSV)

        val hsvMat = if (injection.crop != null) hsvMatUncropped.submat(injection.crop) else hsvMatUncropped

        var high : Double = 0.0
        var detectionToReturn: Pair<String, Vector2>? = null

        for ((iName, color) in injection.signals){
            val (area, position) = color.calculate(hsvMat)

//            println("$name: $area")

            contours.addAll(color.contours)

            //Check if the detection is in the minimum radius
            //Also check if its higher than the previous highest area
            if (color.detect(area) && high < area) {
                high = area
                detectionToReturn = Pair(iName, position)
            }
        }

        if (detectionToReturn != null) {
            detection = Detection(detectionToReturn.first, detectionToReturn.second)
        }

        Imgproc.drawContours(input, contours, -1, Scalar(250.0,0.0,250.0), 2)

        //Release so we don't get any memory leaks
        hsvMatUncropped.release()
        hsvMat.release()
        return if (injection.crop != null) input.submat(injection.crop) else input
    }

    fun getDetection(): Detection? {
        return detection
    }
}