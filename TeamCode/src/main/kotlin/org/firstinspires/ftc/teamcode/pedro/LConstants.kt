package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.localization.Encoder
import com.pedropathing.localization.GoBildaPinpointDriver
import com.pedropathing.localization.constants.PinpointConstants
import com.pedropathing.localization.constants.ThreeWheelConstants
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.motion.Configs

object LConstants {
    init {
        ThreeWheelConstants.forwardTicksToInches = 0.00296843400339
        ThreeWheelConstants.strafeTicksToInches = 0.00296843400339
        ThreeWheelConstants.turnTicksToInches = 0.00296843400339
        ThreeWheelConstants.leftY = Configs.Main.leftOffset.inch
        ThreeWheelConstants.rightY = -Configs.Main.rightOffset.inch
        ThreeWheelConstants.strafeX = -Configs.Main.backOffset.inch
        ThreeWheelConstants.leftEncoder_HardwareMapName = "backRight"
        ThreeWheelConstants.rightEncoder_HardwareMapName = "slidesR"
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "slidesL"
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD

        PinpointConstants.forwardY = Configs.Main.leftOffset.inch
        PinpointConstants.strafeX = -Configs.Main.backOffset.inch
        PinpointConstants.distanceUnit = DistanceUnit.INCH
        PinpointConstants.hardwareMapName = "pinpoint"
        PinpointConstants.useYawScalar = false
        PinpointConstants.yawScalar = 1.0
        PinpointConstants.useCustomEncoderResolution = false
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        PinpointConstants.customEncoderResolution = 13.26291192
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD
    }
}