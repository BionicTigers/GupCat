package org.firstinspires.ftc.teamcode.utils.command

/**
 * Continuously run the command while the provided predicate is true
 */
class ConditionalCommand(callback: () -> Unit, val predicate: () -> Boolean) : Command(callback) {
    override fun run() {
        if (predicate.invoke()) {
            callback.invoke()
        } else {
            remove()
        }
    }
}