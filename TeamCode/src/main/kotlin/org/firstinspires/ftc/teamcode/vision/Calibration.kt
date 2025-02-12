package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl.PanTiltHolder
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import io.github.bionictigers.axiom.utils.Time
import org.openftc.easyopencv.OpenCvWebcam
import java.util.concurrent.TimeUnit

data class Calibration(
    val exposureMode: ExposureControl.Mode = ExposureControl.Mode.Auto,
    val exposureDuration: Time? = null,

    val gain: Int? = null,

    val zoom: Int? = null,
    val panTilt: Pair<Int, Int>? = null,

    val focusMode: FocusControl.Mode = FocusControl.Mode.Auto,
    val focusLength: Double? = null,

    val whiteBalanceMode: WhiteBalanceControl.Mode = WhiteBalanceControl.Mode.AUTO,
    val whiteBalanceTemperature: Int? = null
) {
    init {
        if (exposureMode == ExposureControl.Mode.Manual && exposureDuration == null) {
            throw IllegalArgumentException("Exposure duration must be provided when exposure mode is manual.")
        }

        if (exposureMode == ExposureControl.Mode.Auto && exposureDuration != null) {
            throw IllegalArgumentException("Exposure duration must not be provided when exposure mode is auto.")
        }

        if (focusMode == FocusControl.Mode.Fixed && focusLength == null) {
            throw IllegalArgumentException("Focus length must be provided when focus mode is manual.")
        }

        if (focusMode == FocusControl.Mode.Auto && focusLength != null) {
            throw IllegalArgumentException("Focus length must not be provided when focus mode is auto.")
        }

        if (whiteBalanceMode == WhiteBalanceControl.Mode.MANUAL && whiteBalanceTemperature == null) {
            throw IllegalArgumentException("White balance temperature must be provided when white balance mode is manual.")
        }

        if (whiteBalanceMode == WhiteBalanceControl.Mode.AUTO && whiteBalanceTemperature != null) {
            throw IllegalArgumentException("White balance temperature must not be provided when white balance mode is auto.")
        }


    }

    internal fun apply(camera: OpenCvWebcam) {
        camera.exposureControl.mode = exposureMode
        exposureDuration?.let { camera.exposureControl.setExposure(it.seconds().toLong(), TimeUnit.SECONDS) }

        gain?.let { camera.gainControl.gain = it }

        zoom?.let { camera.ptzControl.zoom = it }
        panTilt?.let {
            val holder = PanTiltHolder()
            holder.pan = panTilt.first
            holder.tilt = panTilt.second

            camera.ptzControl.panTilt = holder
        }

        camera.focusControl.mode = focusMode
        focusLength?.let { camera.focusControl.focusLength = it }

        camera.whiteBalanceControl.mode = whiteBalanceMode
        whiteBalanceTemperature?.let { camera.whiteBalanceControl.whiteBalanceTemperature = it }
    }
}