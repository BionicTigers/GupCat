package org.firstinspires.ftc.teamcode.utils.command

class OnceCommand(callback: () -> Unit) : Command(callback) {
    override fun run() {
        callback.invoke()
        remove()
    }
}