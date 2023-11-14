package org.firstinspires.ftc.teamcode.utils.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

data class AprilTag(val id: Int, val name: String)

class AprilTags(val robot: Robot, hardwareMap : HardwareMap) {

    // get measurement from bot
    private val camPosX = 0.0 // in mm, camera position from the middle left/right
    private val camPosY = 0.0 // in mm, camera position from the middle forward/back

    // rotation is counterclockwise 0 being facing 5040s side
    // x and y are from the bottom left corner while facing 5040's side
    private val aprilTagData = arrayOf(
        Pair(Pose(3437.6, 2914.6, 90.0), "Blue Left"), // poses in mm, mm, deg
        Pair(Pose(3437.6, 2861.1, 90.0), "Blue Mid"),
        Pair(Pose(3437.6, 2807.6, 90.0), "Blue Right"),
        Pair(Pose(3437.6, 850.0, 90.0), "Red Left"),
        Pair(Pose(3437.6, 796.5, 90.0), "Red Mid"),
        Pair(Pose(3437.6, 743.0, 90.0), "Red Right"),
        Pair(Pose(0.0, 750.0, 270.0), "Red Wall"),
        Pair(Pose(0.0, 2907.6, 270.0), "Blue Wall")
    )

    private val aprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .build()
    private var aTPon = true //TODO: CHANGE THIS

    private val visionPortal = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        .addProcessor(aprilTagProcessor)
        .enableCameraMonitoring(true)         // Enable LiveView (RC preview)
        .setAutoStopLiveView(true)            // Automatically stop LiveView (RC preview) when all vision processors are disabled.
        .build()
    private var lVon = true

    private val currentDetections : List<AprilTagDetection> = aprilTagProcessor.detections // an array of the aptgs the camera sees

    // returns the list of aptgs the camera sees when called (id and name)
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

    // returns the final global robot pose and sets the robot's pose to that pose (fgrp is the avg of the grps found from each aptg)
    fun calculateRobotPos(): Pose { // none of this code works if the aptgs rotations are anything other than 90 or 270 :3
        val poses : ArrayList<Pose> = arrayListOf()

        // for each aptg the camera sees, find the global pose of the robot and put each pose in an array
        for (currentDetection in aprilTagProcessor.detections) {
            val globalPose = aprilTagData[currentDetection.id - 1].first // pose of the aptg from aprilTagData array
            if (currentDetection.ftcPose == null) {
                println("NULL")
                continue
            }
            val range = currentDetection.ftcPose.range * 2.54 // inches converted into mm, direct dist from the center of the aptg to the camera
            val bearing = currentDetection.ftcPose.bearing // deg, how much the robot would have to rotate to directly face the aptg
            val yaw = currentDetection.ftcPose.yaw // deg, rotation of the tag away or towards the camera
            val rawX = currentDetection.ftcPose.x // x val from the camera

            // finding the x and y that line up with the x and y lines on the field
            val dirX = globalPose.x + (range * sin((90 - bearing) + yaw))
            val dirY = globalPose.y + (range * cos((90 - bearing) + yaw))

            // finding global robot rotation (yaw might not be neg idk yet)
            val robotRot = globalPose.rotation + (-yaw)

            // finding center of robot from cameras pos
            val radius = ( 2.0.pow(camPosX) + 2.0.pow(camPosY) ).pow(.5)

            val angle = if (robotRot >= 270)
                (robotRot - 270) + asin(camPosX/radius)
            else
                robotRot + 90 + asin(camPosX/radius)

            val camToMidX = if (robotRot < 180)
                -(cos(angle) * radius)
            else
                cos(angle) * radius

            val camToMidY = if (robotRot < 90 || robotRot > 270)
                -(sin(angle) * radius)
            else
                sin(angle) * radius

            // finding global y pos of the robot :(
            val angRadX = acos(rawX/range)

            val robotY = if (robotRot < 90 || robotRot > 180 && robotRot < 270) { // 1
                if (90 - abs(yaw) > angRadX)
                    globalPose.y - dirX + camToMidY
                else
                    globalPose.y + dirX + camToMidY

            } else if ((robotRot > 90 && robotRot < 180) || robotRot > 270) {
                if (90 - abs(yaw) < angRadX)
                    globalPose.y - dirX + camToMidY
                else
                    globalPose.y + dirX + camToMidY

            } else {
                if (robotRot == 90.0) {
                    if (rawX < 0)
                        globalPose.y - dirX + camToMidY
                    else
                        globalPose.y + dirX + camToMidY
                } else {
                    if (rawX > 0)
                        globalPose.y - dirX + camToMidY
                    else
                        globalPose.y + dirX + camToMidY
                }
            }

            // finding global x pos of the robot
            val robotX = if (globalPose.rotation == 90.0)
                globalPose.x - dirY + camToMidX
            else
                globalPose.x + dirY + camToMidX

            // makes pose
            val botPose = Pose(robotX, robotY, robotRot)
            poses.add(botPose)
        }


        var newPose = Pose()

        poses.forEach { pose ->
            newPose += pose
        }
        newPose /= poses.size.toDouble()

        robot.pose = newPose

        return newPose
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

    fun aprilTagLog(finalPose: Pose, telemetry: Telemetry) { // outputs id, name, xy coords, and range, bearing, yaw, calculated pose for each aptg and the final pose
        telemetry.addData(currentDetections.size.toString(), " apriltags detected")

        for (currentDetection in aprilTagProcessor.detections) {
            if (currentDetection.metadata != null) {

                val cdid = currentDetection.id
                aprilTagData[(cdid - 1)].second

                telemetry.addLine("ID: $cdid Name: ${aprilTagData[(cdid - 1)].first}")
                telemetry.addLine("Global Field Pos: ${aprilTagData[(cdid - 1)].second}")

                telemetry.addLine("  X: ${currentDetection.ftcPose.x} Y: ${currentDetection.ftcPose.y}")
                telemetry.addLine("  Yaw ${currentDetection.ftcPose.yaw} (deg)") // rotation of the tag away or towards the camera
                telemetry.addLine("  Range: ${currentDetection.ftcPose.range} Bearing: ${currentDetection.ftcPose.bearing} (inch, deg)/n")
                /* Range, direct (point-to-point) distance to the tag center
                Bearing, the angle the camera must turn (left/right) to point directly at the tag center
                */
            } else {
                telemetry.addLine("no metadata for apriltags")
            }
        }

        telemetry.addLine("Final Pose: $finalPose") // prints final pose from calculateRobotPos
    }

}