package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Helpful values used in the pipeline
 */
private data object Values {
    val tiltAngle = Math.toRadians(30.0)
    val cameraHeight = Distance.inch(10.0)

    object Samples {
        val length = Distance.inch(3.5)
        val width = Distance.inch(1.5)
        val height = Distance.inch(1.5)

        val yellow = ColorRange(Color(117, 109, 22), Color(255, 244, 125))
        val red = ColorRange(Color(100, 100, 0), Color(255, 255, 50))
        val blue = ColorRange(Color(0, 0, 0), Color(0, 0, 0))
    }
}

/**
 * A pipeline for detecting objects
 *
 * @param camera The camera to use
 */
class Pipeline(private val camera: Camera) : OpenCvPipeline() {
    /**
     * Get the contours of a color range
     *
     * @param input The input image
     * @param range The color range
     *
     * @return The contours of the color range
     */
    private fun getColorRangeContours(input: Mat, range: ColorRange): List<MatOfPoint> {
        val mask = Mat()
        Core.inRange(input, range.lower.scalar, range.upper.scalar, mask)

        val contours = mutableListOf<MatOfPoint>()
        Imgproc.findContours(mask, contours, Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)

        mask.release()

        return contours
    }

    /**
     * Get the contours of a color range group
     *
     * @param input The input image
     * @param range The color range group
     *
     * @return The contours of the color range group
     */
    private fun getColorRangeContours(input: Mat, range: ColorRangeGroup): List<MatOfPoint> {
        val allContours = mutableListOf<MatOfPoint>()

        for (color in range) {
            allContours.addAll(getColorRangeContours(input, color))
        }

        return allContours
    }

    /**
     * Gather the contours of the sample objects
     *
     * @param input The input image
     *
     * @return The contours of the sample objects
     */
    private fun gatherSampleContours(input: Mat): Triple<List<MatOfPoint>, List<MatOfPoint>, List<MatOfPoint>> {
        val yellowContours = getColorRangeContours(input, Values.Samples.yellow)
        val redContours = getColorRangeContours(input, Values.Samples.red)
        val blueContours = getColorRangeContours(input, Values.Samples.blue)

        return Triple(yellowContours, redContours, blueContours)
    }

    /**
     * Calculate the distance of a contour from the camera relative to the camera's position
     *
     * @param input The input image
     * @param contour The contour to calculate the distance of
     * @param length The length of the object
     * @param width The width of the object
     *
     * @return The location of the contour relative to the camera
     */
    private fun calculateLocationOfContour(input: Mat, contour: MatOfPoint, length: Distance, width: Distance, height: Distance): Vector2 {
        val closestPoint = contour.toList().minBy { it.y }
        val center = Vector2(contour.toList().sumOf { it.x } / contour.toList().size, contour.toList().sumOf { it.y } / contour.toList().size)

        val distanceCamera = (camera.focalLength * length) / closestPoint.y
        val distanceHorizontal = distanceCamera * cos(Values.tiltAngle)

        val distanceField = (distanceHorizontal.pow(2.0) + Values.cameraHeight.pow(2.0)).sqrt()
        val angle = ((center.x - (camera.width / 2)) / camera.width) * camera.fov

        return Vector2(distanceField.mm * cos(angle), distanceField.mm * sin(angle))
    }

    /**
     * Crop the input image
     *
     * @param input The input image
     *
     * @return The cropped image
     */
    private fun crop(input: Mat): Mat {
        return if (camera.crop != null) input.submat(camera.crop) else input
    }

    /**
     * Process a frame
     *
     * @param input The input image
     *
     * @return The processed image
     */
    override fun processFrame(input: Mat?): Mat {
        if (input == null) return Mat()

        val cropped = crop(input)
        val (yellowContours, redContours, blueContours) = gatherSampleContours(cropped)

        Imgproc.drawContours(cropped, yellowContours, -1, Scalar(255.0, 255.0, 0.0), 2)
        Imgproc.drawContours(cropped, redContours, -1, Scalar(255.0, 0.0, 0.0), 2)
        Imgproc.drawContours(cropped, blueContours, -1, Scalar(0.0, 0.0, 255.0), 2)

        return cropped
    }
}