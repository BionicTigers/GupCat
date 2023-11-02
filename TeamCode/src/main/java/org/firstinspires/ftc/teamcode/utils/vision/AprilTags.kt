package org.firstinspires.ftc.teamcode.utils.vision

import androidx.core.graphics.component1
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.cos
import kotlin.math.sin

data class AprilTag(val id: Int, val name: String)

class AprilTags(private val robot: Robot, hardwareMap : HardwareMap) { // :3
    val aprilTagData = arrayOf(
        Pair(Pose(3437.6, 2914.6, 90.0), "Blue Left"), // poses in mm, mm, deg
        Pair(Pose(3437.6, 2861.1, 90.0), "Blue Mid"),
        Pair(Pose(3437.6, 2807.6, 90.0), "Blue Right"),
        Pair(Pose(3437.6, 850.0, 90.0), "Red Left"),
        Pair(Pose(3437.6, 796.5, 90.0), "Red Mid"),
        Pair(Pose(3437.6, 743.0, 90.0), "Red Right"),
        Pair(Pose(0.0, 750.0, 270.0), "Red Wall"),
        Pair(Pose(0.0, 2907.6, 270.0), "Blue Wall")
    )

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

    val currentDetections : List<AprilTagDetection> = aprilTagProcessor.detections

    fun getAprilTagDetections(): Array<AprilTag> {
        val list = ArrayList<AprilTag>()

        for (currentDetection in aprilTagProcessor.detections) {
            if (currentDetection.metadata != null) {
                list.add(
                    AprilTag(
                        currentDetection.id,
                        aprilTagData[(currentDetection.id - 1)].second
                    )
                )
            }
        }

        return list.toTypedArray()
    }

    fun calculateRobotPos() {
        var poses : ArrayList<Pose> = arrayListOf()

        for (currentDetection in aprilTagProcessor.detections) {

            val globalPose = aprilTagData[currentDetection.id - 1].first // pose from aprilTagData array

            val range = currentDetection.ftcPose.range * 2.54 // inches converted into mm
            val bearing = currentDetection.ftcPose.bearing // deg
            val yaw = currentDetection.ftcPose.yaw // deg

            // finding global positions (not finished)
            val robotX = globalPose.x + (range * sin((90 - bearing) + yaw))
            val robotY = globalPose.y + (range * cos((90 - bearing) + yaw))
            val robotRot = yaw

            val botPose = Pose(robotX, robotY, robotRot)
            poses.add(botPose)
        }


        var newPose = Pose()

        poses.forEach { pose ->
            newPose += pose
        }
        newPose /= poses.size.toDouble()

        robot.pose = newPose
    }

    fun toggleLiveView() { // saves CPU resources while off
        if (lVon)
            visionPortal.stopLiveView()
        else
            visionPortal.resumeLiveView()

        lVon = !lVon
    }

    fun toggleATProcessor() { // saves more CPU resources while off
        if (aTPon)
            visionPortal.setProcessorEnabled(aprilTagProcessor, false)
         else
            visionPortal.setProcessorEnabled(aprilTagProcessor, true)


        aTPon = !aTPon
    }

//    fun aprilTagLog(poses: ArrayList<Pose>, finalPose: Pose) { // outputs id, name, xy coords, and range, bearing, and elevation for each aptg
//        telemetry.addData(currentDetections.size.toString(), " Apriltags detected")
//
//        for (currentDetection in aprilTagProcessor.detections) {
//            if (currentDetection.metadata != null) {
//                telemetry.addLine("ID: ${currentDetection.id} Name: ${currentDetection.metadata.name}")
//                telemetry.addLine("Global Field Pos: ${currentDetection.metadata.fieldPosition}")
//
//                telemetry.addLine("  X: ${currentDetection.ftcPose.x} Y: ${currentDetection.ftcPose.y}")
//                telemetry.addLine("  Yaw ${currentDetection.ftcPose.yaw}") // rotation of the tag away or towards the camera
//                telemetry.addLine("  Range: ${currentDetection.ftcPose.range} Bearing: ${currentDetection.ftcPose.bearing} (inch, deg)/n")
//                /* Range, direct (point-to-point) distance to the tag center
//                Bearing, the angle the camera must turn (left/right) to point directly at the tag center
//                */
//            } else {
//                telemetry.addLine("no metadata for apriltags :( (telemetry)")
//            }
//        }
//        telemetry.addLine("Poses:")
//        for (pose in poses) { // prints each pose from calculateRobotPos
//            telemetry.addLine(" $pose")
//        }
//
//        telemetry.addLine("Final Pose: $finalPose") // prints final pose from calculateRobotPos
//    }

}