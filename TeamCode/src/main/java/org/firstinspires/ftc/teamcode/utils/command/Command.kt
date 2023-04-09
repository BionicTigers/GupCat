package org.firstinspires.ftc.teamcode.utils.command

abstract class Command(internal val callback: () -> Unit) {
    val priority: Int?
        get() {
            return Scheduler.getPriority(this)
        }

    abstract fun run()

    /**
     * If the command is in the scheduler, remove it.
     */
    fun remove() {
        priority?.let { Scheduler.remove(it) }
    }
}