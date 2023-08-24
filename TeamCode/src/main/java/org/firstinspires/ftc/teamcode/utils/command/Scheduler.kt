package org.firstinspires.ftc.teamcode.utils.command

import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Task Scheduler used to maintain the order of code execution
 */
object Scheduler {
    private val schedule: ArrayList<Command?> = ArrayList()
    private val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    var deltaTime = 0.0
        internal set

    /**
     * Run's all the commands in order of priority
     */
    fun update() {
        deltaTime = elapsedTime.milliseconds()

        for (command in schedule.toArray()) {
            command?.let {(it as Command).run()}
        }

        elapsedTime.reset()
    }
    /**
     *Removes a command at a given index from the scheduler
     */
    fun remove(priority: Int) {
        schedule[priority] = null
    }

    /**
     *Removes a command from the scheduler
     */
    fun remove(command: Command) {
        command.priority?.let {schedule[it] = null}
    }

    /**
     * Removes a command at a given index from the scheduler while pushing the index's ahead back one.
     */
    fun removeAndPush(priority: Int) {
        schedule.removeAt(priority)
    }

    /**
     * Removes a command from the scheduler while pushing the index's ahead back one.
     */
    fun removeAndPush(command: Command) {
        command.priority?.let {schedule.removeAt(it)}
    }

    /**
     * Adds a command to the next free index
     */
    fun add(command: Command) {
        schedule.add(command)
    }

    /**
     * Adds a command to the scheduler, pushing forward the element it replaces (if any) and any that are connected to it.
     */
    fun add(command: Command, priority: Int) {
        schedule.add(priority, command)
    }

    /**
     *  Returns the first found priority of the passed in command.
     *
     *  If not found, returns null.
     */
    fun getPriority(command: Command): Int? {
        return schedule.indexOf(command).takeIf { it != -1 }
    }

    fun clear() {
        schedule.clear()
    }
}