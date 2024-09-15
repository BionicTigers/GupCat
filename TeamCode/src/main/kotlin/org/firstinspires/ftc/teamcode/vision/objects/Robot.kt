package org.firstinspires.ftc.teamcode.vision.objects

import org.firstinspires.ftc.teamcode.utils.Pose

data class Robot(override val position: Pose, val alliance: Alliance) : BaseObject(position) {
    enum class Alliance {
        Red,
        Blue
    }
}