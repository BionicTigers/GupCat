package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.Distance
import org.opencv.core.Rect
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.atan2

class Camera(webcamName: WebcamName, monitorId: Int? = null, val rotation: OpenCvCameraRotation = OpenCvCameraRotation.UPRIGHT, val width: Int = 1280, val height: Int = 720, val calibration: Calibration = Calibration()) {
    private val camera = if (monitorId != null)
        OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorId)
    else
        OpenCvCameraFactory.getInstance().createWebcam(webcamName)

    var crop: Rect? = null

    val focalLength: Distance
        get() = Distance.mm(camera.focusControl.focusLength)
    val fov: Double
        get() = 2 * atan2(height / 2.0, focalLength.mm).toDegrees()

    fun startCameraStream(pipeline: OpenCvPipeline) {
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                calibration.apply(camera)
                camera.startStreaming(width, height, rotation)
                camera.showFpsMeterOnViewport(true)
                camera.setPipeline(pipeline)
            }

            override fun onError(errorCode: Int) {
                throw RuntimeException("Camera Initialization Failed: $errorCode")
            }
        })
    }
}

fun Number.toDegrees() = Math.toDegrees(this.toDouble())
fun Number.toRadians() = Math.toRadians(this.toDouble())