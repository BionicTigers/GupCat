package org.firstinspires.ftc.teamcode.utils.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.cos
import kotlin.math.sin

class AprilTags(hardwareMap : HardwareMap) {
    // Set custom features of the Processor (Optional)
    val aprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawTagID(true)          // Default is true
        .setDrawTagOutline(true)     // Default is true
        .setDrawAxes(true)           // Default is false
        .setDrawCubeProjection(true) // Default is false
        .build()
    var aTPon = true

    // Set custom features of the Vision Portal (Optional)
    val visionPortal = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        .addProcessor(aprilTagProcessor)
        .enableCameraMonitoring(true)         // Enable LiveView (RC preview)
        .setAutoStopLiveView(true)            // Automatically stop LiveView (RC preview) when all vision processors are disabled.
        .build()
    var lVon = true



    val currentDetections : List<AprilTagDetection> = aprilTagProcessor.getDetections()

    fun aprilTagTelemetry(telemetry: Telemetry) { // outputs id, name, xy coords, and range, bearing, and elevation for each aptg
        telemetry.addData(currentDetections.size.toString(), " Apriltags detected")

        for (currentDetection in aprilTagProcessor.getDetections()) {
            if (currentDetection.metadata != null) {
                telemetry.addLine("ID: ${currentDetection.id} Name: ${currentDetection.metadata.name}")
                telemetry.addLine("Global Field Pos: ${currentDetection.metadata.fieldPosition}")

                telemetry.addLine("  X: ${currentDetection.ftcPose.x} Y: ${currentDetection.ftcPose.y}")
                telemetry.addLine("  Yaw ${currentDetection.ftcPose.yaw}") // rotation of the tag away or towards the camera
                telemetry.addLine("  Range: ${currentDetection.ftcPose.range} Bearing: ${currentDetection.ftcPose.bearing} (inch, deg)/n")
                /* Range, direct (point-to-point) distance to the tag center
                Bearing, the angle the camera must turn (left/right) to point directly at the tag center
                */
            } else {
                telemetry.addLine("no metadata for apriltags :( (telemetry)")
            }
        }
    }

    fun calculateRobotPos(telemetry: Telemetry): Pose {
        var poses : ArrayList<Pose> = arrayListOf()

        for (currentDetection in aprilTagProcessor.getDetections()) {
            if (currentDetection.metadata != null) {
                val globalX = currentDetection.metadata.fieldPosition.get(0) // in mm from field center
                val globalY = currentDetection.metadata.fieldPosition.get(1) // in mm from field center

                val range = currentDetection.ftcPose.range * 2.54 // inches converted into mm
                val bearing = currentDetection.ftcPose.bearing // deg
                val yaw = currentDetection.ftcPose.yaw //deg

                val robotX : Double = globalX + (range * sin((90 - bearing) + yaw))
                val robotY : Double = globalY + (range * cos((90 - bearing) + yaw))
                val robotRot : Double = yaw

                val botPose : Pose = Pose(robotX, robotY, robotRot)
                poses.add(botPose)

                telemetry.addLine("Robot Pos: $botPose (from ${currentDetection.metadata.name})")
            } else {
                telemetry.addLine("no metadata for apriltags :( (from ${currentDetection.metadata.name})")
            }
        }

        val botPoseFinal : Pose = Pose()

        return botPoseFinal
    }

    fun toggleLiveView() { // saves CPU resources while off
        if (lVon) {
            visionPortal.stopLiveView()
        } else {
            visionPortal.resumeLiveView()
        }

        lVon = !lVon
    }

    fun toggleATProssessor() { // saves CPU resources while off
        if (aTPon) {
            visionPortal.setProcessorEnabled(aprilTagProcessor, false)
        } else {
            visionPortal.setProcessorEnabled(aprilTagProcessor, true)
        }

        aTPon = !aTPon
    }
}