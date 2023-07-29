package org.firstinspires.ftc.teamcode.utils.vision

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.Rect
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


class OpenCv(webcamName: WebcamName, val signals: HashMap<String, Color>, monitorId: Int? = null) {
    private var camera: OpenCvWebcam
    var crop: Rect? = null
    private var pipeline: Pipeline
    //Create a new OpenCV Wrapper WITH a live monitor view
    //Useful for debugging but slows down cpu cycles

    init {
        if (monitorId != null)
            this.camera = OpenCvCameraFactory.getInstance()
                .createWebcam(webcamName, monitorId)
        else
            this.camera = OpenCvCameraFactory.getInstance()
                .createWebcam(webcamName)

        //Use dependency injection for real-time updating
        pipeline = Pipeline(this)

        startCameraStream()
    }


    //Return the current detection from the pipeline, NONE if there isn't a detection
    fun getDetection(): String? {return pipeline.getDetection()}

    fun stopDetection() {camera.stopStreaming()}


    private fun startCameraStream () {
        //Start the camera asynchronously to prevent yielding
        //This creates a new thread but it won't cause any issues with hardware ownership
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener{
            override fun onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
                camera.setPipeline(pipeline)
            }

            override fun onError(errorCode: Int) {
                throw RuntimeException(String.format("Camera Initialization Failed: %d", errorCode))
            }
        })


    }
}