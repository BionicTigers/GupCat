package org.firstinspires.ftc.teamcode.utils.vision

import android.annotation.SuppressLint
import android.graphics.Rect
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam
import java.util.concurrent.TimeUnit

public final class Pipeline : OpenCvPipeline() {
    var injection: OpenCv? = null
    private val detection = "None"

    fun Pipeline(injection: OpenCv?) { this.injection = injection }

    //This is essentially the Main method for the pipeline
    //On every new frame from the camera, this is ran
    override fun processFrame(input: Mat?): Mat {
        val hsvMatUncropped = Mat()

        val contours = ArrayList<MatOfPoint>()

        //Convert the input image into HSV
        Imgproc.cvtColor(input, hsvMatUncropped, Imgproc.COLOR_RGB2HSV)

        val hsvMat =
            hsvMatUncropped.submat(injection!!.crop)

        //Highest Area found during every signal process
        val high: Double = 0.0

        for (entry in injection!!.signals.entries){

        }
    }
}
class OpenCv {
    private lateinit var camera: OpenCvWebcam
    lateinit var signals: HashMap<String, Color>
    lateinit var crop: Rect
    private lateinit var pipeline: Pipeline
    //Create a new OpenCV Wrapper WITH a live monitor view
    //Useful for debugging but slows down cpu cycles

    //thi can be removed
    @SuppressLint("NotConstructor")


    fun OpenCv(webcamName: WebcamName, signals: HashMap<String, Color>, monitorId: Int ){
        this.camera = OpenCvCameraFactory.getInstance()
            .createWebcam(webcamName, monitorId)

        this.signals = signals

        //Use dependency injection for real-time updating
        pipeline = pipeline(this)

        startCameraStream()
    }

    fun setCrop(zone: Rect) {crop = zone}

    //Return the current detection from the pipeline, NONE if there isn't a detection
    fun getDetection() {return pipeline.getDetection}

    fun stopDetection() {camera.stopStreaming()}

    private fun startCameraStream () {
        //Start the camera asynchronously to prevent yielding
        //This creates a new thread but it won't cause any issues with hardware ownership
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener{
            @Override
            override fun onOpened() {
                camera.getExposureControl().setMode(ExposureControl.Mode.Manual)
                camera.exposureControl.setExposure(VisionConstants.EXPOSURE, TimeUnit.MILLISECONDS)
                //Start streaming at 1280x720, in the upright orientation
                //Start the detection pipeline
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
                camera.setPipeline(pipeline)

                //Used to give realtime camera feed to FTC Dashboard
            }

            @Override
            override fun onError(errorCode: Int){
                throw RuntimeException(String.format("Camera Initialization Failed: %d", errorCode))
            }

        })


    }
}

