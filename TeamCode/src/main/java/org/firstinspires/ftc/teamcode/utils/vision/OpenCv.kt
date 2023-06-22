package org.firstinspires.ftc.teamcode.utils.vision

import android.annotation.SuppressLint
import android.graphics.Rect
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvWebcam
import java.util.HashMap
import java.util.SimpleTimeZone


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
        camera.openCameraDeviceAsync({
            @Override
            fun onOpened() {
                camera.getExposureControl().setMode(ExposureControl.Mode.Manual)
            }
        })


    }
}