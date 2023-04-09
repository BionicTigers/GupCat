package org.firstinspires.ftc.teamcode.utils.command

/**
 * Continuously run the command, can only be stopped manually
 */
class ContinuousCommand(callback: () -> Unit) : Command(callback) {
    override fun run() {
        callback.invoke()
    }
}