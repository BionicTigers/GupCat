package org.firstinspires.ftc.teamcode.utils.vision

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam
import java.util.concurrent.TimeUnit

class Pipeline : OpenCvPipeline{
    var injection: Mat.firstinspires.ftc.teamcode.util.OpenCv? = null

    private var detection = "None"

    fun Pipeline(injection: org.firstinspires.ftc.teamcode.util.OpenCv?) {
        this.injection = injection
    }

    //This is essentially the Main method for the pipeline
    //On every new frame from the camera, this is ran
    override fun processFrame(input: Mat): Mat? {
        val hsvMatUncropped = Mat()
        val contours = ArrayList<MatOfPoint>()
        //Convert the input image into HSV
        Imgproc.cvtColor(input, hsvMatUncropped, Imgproc.COLOR_RGB2HSV)
        val hsvMat =
            if (injection.crop != null) hsvMatUncropped.submat(injection.crop) else hsvMatUncropped

        //Highest Area found during every signal process
        var high = 0.0
        for ((name, signal): Map.Entry<String, Signal> in injection.signals.entries) {
            val area: Double = signal.getArea(hsvMat)
            println("$name: $area")
            contours.addAll(signal.contours)

            //Check if the detection is in the minimum radius
            //Also check if its higher than the previous highest area
            if (signal.detect(area) && high < area) {
                high = area
                detection = name
            }
        }

        //Draw contours on top of the image for easier debugging
        //This is a work in progress.
        Imgproc.drawContours(input, contours, -1, Scalar(250.0, 0.0, 250.0), 2)

        //Release so we don't get any memory leaks
        hsvMatUncropped.release()
        hsvMat.release()
        return if (injection.crop != null) input.submat(injection.crop) else input
    }

    fun getDetection(): String? {
        return detection
    }
}

public class OpenCv {
    private var camera: OpenCvWebcam? = null
    var signals: HashMap<String, Signal>? = null
    var crop: Rect? = null
    private var pipeline: Pipeline? = null

    //Create a new OpenCV Wrapper WITH a live monitor view
    //Useful for debugging but slows down cpu cycles
    fun OpenCv(webcamName: WebcamName?, signals: HashMap<String, Signal>?, monitorId: Int) {
        camera = OpenCvCameraFactory.getInstance()
            .createWebcam(webcamName, monitorId)
        this.signals = signals

        //Use dependency injection for real-time updating
        pipeline = Pipeline(this)
        startCameraStream()
    }

    //Create a new OpenCV Wrapper WITHOUT a live monitor view
    fun OpenCv(webcamName: WebcamName?, signals: HashMap<String, Signal>?) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName)
        this.signals = signals
        pipeline = Pipeline(this)
        startCameraStream()
    }

    fun setCrop(zone: Rect?) {
        crop = zone
    }

    //Return the current detection from the pipeline, NONE if there isn't a detection
    fun getDetection(): String? {
        return pipeline!!.getDetection()
    }

    fun stopDetection() {
        camera!!.stopStreaming()
    }

    private fun startCameraStream() {
        //Start the camera asynchronously to prevent yielding
        //This creates a new thread but it won't cause any issues with hardware ownership
        camera!!.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera!!.exposureControl.mode = ExposureControl.Mode.Manual
                camera!!.exposureControl.setExposure(
                    VisionConstants.EXPOSURE,
                    TimeUnit.MILLISECONDS
                )
                //Start streaming at 1280x720, in the upright orientation
                //Start the detection pipeline
                camera!!.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
                camera!!.setPipeline(pipeline)

            }

            override fun onError(errorCode: Int) {
                throw RuntimeException(String.format("Camera Initialization Failed: %d", errorCode))
            }
        })
    }
}