package org.firstinspires.ftc.teamcode.vision.objects

import java.util.UUID

class Registry {
    val objects = hashMapOf<UUID, BaseObject>()
    val robots = hashMapOf<UUID, Robot>()
    val samples = hashMapOf<UUID, Sample>()

    fun addSample(id: UUID, sample: Sample) {
        samples[id] = sample
        objects[id] = sample
    }

    //Robot should be the only one used right now
    fun addRobot(id: UUID, robot: Robot) {
        robots[id] = robot
        objects[id] = robot
    }

    fun removeObject(id: UUID) {
        objects.remove(id)
        robots.remove(id)
        samples.remove(id)
    }

    fun clearObjects() {
        objects.clear()
    }
}