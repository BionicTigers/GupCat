package org.firstinspires.ftc.teamcode.utils.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

data class AprilTag(val id: Int, val name: String)

class AprilTags(val robot: Robot, hardwareMap : HardwareMap) {

    // get measurement from bot  **make it negative if the distance is negative when placed on the field** (neg if camera is to the left or down, pos if camera is right or up)
    private val camPosX = 0.0 // in mm, camera position from the middle left/right
    private val camPosY = 0.0 // in mm, camera position from the middle forward/back

    // rotation is counterclockwise 0 being facing 5040s side (this is different from the robot's rotation, which is clockwise)
    // x and y are from the bottom left corner while facing 5040's side
    private val aprilTagData = arrayOf(
        Pair(Pose(3437.6, 2914.6, 90.0), "Blue Left"), // poses in mm, mm, deg
        Pair(Pose(3437.6, 2861.1, 90.0), "Blue Mid"),
        Pair(Pose(3437.6, 2807.6, 90.0), "Blue Right"),
        Pair(Pose(3437.6, 850.0, 90.0), "Red Left"),
        Pair(Pose(3437.6, 796.5, 90.0), "Red Mid"),
        Pair(Pose(3437.6, 743.0, 90.0), "Red Right"),
        Pair(Pose(0.0, 750.0, 270.0), "Red Wall Big"),
        Pair(Pose(0.0, 895.35, 270.0), "Red Wall Small"),
        Pair(Pose(0.0, 2767.9, 270.0), "Blue Wall Small"),
        Pair(Pose(0.0, 2907.6, 270.0), "Blue Wall Big")
    )

    private val aprilTagProcessor = AprilTagProcessor.Builder()
        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setLensIntrinsics(815.193, 815.193, 309.116, 289.874) // values from calibrating the camera
        .build()
    private var processorOn = true

    private val visionPortal = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        .addProcessor(aprilTagProcessor)
        .enableLiveView(true)         // enable LiveView
        .setAutoStopLiveView(true)    // automatically stop LiveView when all vision processors are disabled
        .build()
    private var liveViewOn = true

    private val currentDetections : List<AprilTagDetection> = aprilTagProcessor.detections // an array of the apriltags the camera sees

    // returns the list of apriltags the camera sees when called (id and name)
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

    fun printEverything() {
        for (currentDetection in aprilTagProcessor.detections) {
            if (currentDetection.rawPose != null) {
                println("id: ${currentDetection.id}")
                println("tag pose: ${aprilTagData[currentDetection.id - 1].first}")
                println("range: ${currentDetection.ftcPose.range}")
                println("bearing: ${currentDetection.ftcPose.bearing}")
                println("yaw: ${currentDetection.ftcPose.yaw}")
                println("x: ${currentDetection.ftcPose.x}")
                println("Final Pose: ${calculateRobotPos()}")
            }
        }
    }

    // returns the final global robot pose and sets the robot's pose to that pose
    fun calculateRobotPos(): Pose { // none of this code works if the apriltags' rotations are anything other than 90 or 270 :3
        val poses : ArrayList<Pose> = arrayListOf()

        // for each apriltags the camera sees, find the global pose of the robot and put each pose in an array
        for (currentDetection in aprilTagProcessor.detections) {
            if (currentDetection.rawPose != null) {
                val globalPose = aprilTagData[currentDetection.id - 1].first // pose of the apriltags from aprilTagData array

                val range = currentDetection.ftcPose.range * 25.4 // inches converted into mm, direct dist from the center of the aptg to the camera
                val bearing = currentDetection.ftcPose.bearing // deg, how much the robot would have to rotate to directly face the aptg
                val yaw = currentDetection.ftcPose.yaw // deg, rotation of the tag away or towards the camera
                val rawX = currentDetection.ftcPose.x // x val from the camera
                // there is a diagram of these values here: https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html

                // finding the x and y from the apriltag to the camera that line up with the global x and y lines on the field (parallel to the walls)
                val dirX = (range * sin(yaw))
                val dirY = (range * cos(yaw))

                // finding global robot rotation
                var robotRot = if (globalPose.rotation == 90.0) {
                    if (yaw < 0) {
                        270 + (bearing + -yaw)
                    } else {
                        270 + (-bearing + -yaw)
                    }
                } else {
                    if (yaw<0) {
                        90 + (bearing + -yaw)
                    } else {
                        90 + (-bearing + -yaw)
                    }
                }

                // finding pos of center of robot from the camera pos
                val h = (2.0.pow(camPosX) + 2.0.pow(camPosY)).pow(.5)

                val camToMidX = sin(robotRot)*camPosY+cos(robotRot)*camPosX

                val camToMidY = (2.0.pow(h) + 2.0.pow(camToMidX)).pow(.5)

                // finding global y pos of the robot
                val angRadX = acos(rawX / range)

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

                // flipping the rotation (im not redoing all that math)
                robotRot = -robotRot + 360

                // makes pose
                val botPose = Pose(robotX, robotY, robotRot)
                poses.add(botPose)
            }
        }

        var newPose = Pose()

        // finds average of all the poses
        poses.forEach { pose ->
            newPose += pose
        }
        newPose /= poses.size.toDouble()

        // sets the robots pose to the final pose
        robot.pose = newPose

        return newPose
    }

    fun toggleLiveView() { // saves CPU resources while off
        if (liveViewOn)
            visionPortal.stopLiveView()
        else
            visionPortal.resumeLiveView()

        liveViewOn = !liveViewOn
    }

    fun toggleATProcessor() { // saves more CPU resources while off
        if (processorOn)
            visionPortal.setProcessorEnabled(aprilTagProcessor, false)
         else
            visionPortal.setProcessorEnabled(aprilTagProcessor, true)


        processorOn = !processorOn
    }

    fun aprilTagLog(finalPose: Pose, telemetry: Telemetry) { // outputs id, name, xy coords, and range, bearing, yaw, calculated pose for each apriltag and the final pose
        telemetry.addData(currentDetections.size.toString(), " apriltags detected")

        for (currentDetection in aprilTagProcessor.detections) {
            if (currentDetection.metadata != null) {

                val currentId = currentDetection.id
                if (currentId == 4) {

                    telemetry.addLine("ID: $currentId Name: ${aprilTagData[(currentId - 1)].first}")
                    telemetry.addLine("Global Field Pos: ${aprilTagData[(currentId - 1)].second}")

                    telemetry.addLine("  X: ${currentDetection.ftcPose.x} Y: ${currentDetection.ftcPose.y}")
                    telemetry.addLine("  Yaw ${currentDetection.ftcPose.yaw} (deg)") // rotation of the tag away or towards the camera
                    telemetry.addLine("  Range: ${currentDetection.ftcPose.range} Bearing: ${currentDetection.ftcPose.bearing} (inch, deg)/n")
                    /* Range: direct distance to the tag center
                    Bearing: the angle the camera must turn (left/right) to point directly at the tag center */
                }
            } else {
                telemetry.addLine("no metadata for apriltags")
            }
        }

        telemetry.addLine("Final Pose: $finalPose") // prints final pose from calculateRobotPos
        telemetry.update()
    }

}