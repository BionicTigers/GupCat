package org.firstinspires.ftc.teamcode.vision

class OpenCV(camera: Camera) {
    private val pipeline = Pipeline(camera)

    init {
        camera.startCameraStream(pipeline)
    }
}