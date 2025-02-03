package org.firstinspires.ftc.teamcode.axiom.commands

import org.firstinspires.ftc.teamcode.utils.Time
import org.firstinspires.ftc.teamcode.utils.max
import java.util.Stack

object Scheduler {
    private val commands = HashMap<Int, Command<*>>()
    private val sortedCommands = ArrayList<Command<*>>()

    private val addQueue: ArrayList<Command<*>> = ArrayList()
    private val removeQueue: ArrayList<Command<*>> = ArrayList()

    private var changed = false

    var loopDeltaTime = Time()

    /**
     * Adds commands to the scheduler.
     *
     * Duplicate commands will be ignored.
     *
     * @param command The commands to add.
     * @see Command
     */
    fun add(vararg command: Command<*>) {
        addQueue.addAll(command)
    }

    fun add(commands: Collection<Command<*>>) {
        addQueue.addAll(commands)
    }

    private fun internalAdd(command: Command<*>) {
        changed = true
        commands[command.hashCode()] = command
    }

    /**
     * Adds systems to the scheduler.
     *
     * This is a convenience method that adds the beforeRun and afterRun commands of the system.
     *
     * @param system The systems to add.
     * @see System
     */
    fun addSystem(vararg system: System) {
        addQueue.addAll(system.mapNotNull { it.beforeRun })
        addQueue.addAll(system.mapNotNull { it.afterRun })
    }

    /**
     * Removes a command from the scheduler.
     *
     * If the command is not in the scheduler, it will be ignored.
     *
     * @param command The command to remove.
     * @see Command
     */
    fun remove(vararg command: Command<*>) {
        command.forEach {
            if (it !in commands.values) {
                return
            }

            removeQueue.add(it)
        }
    }

    private fun internalRemove(command: Command<*>) {
        changed = true
        commands.remove(command.hashCode())
    }

    private fun sort() {
        val visited = HashSet<Command<*>>()
        val stack = Stack<Command<*>>()

        for (command in commands.values) {
            if (command !in visited) {
                topologicalSort(command, visited, stack)
            }
        }

        sortedCommands.clear()
        while (stack.isNotEmpty()) {
            sortedCommands.add(stack.pop())
        }
    }

    private fun topologicalSort(command: Command<*>, visited: HashSet<Command<*>>, stack: Stack<Command<*>>) {
        visited.add(command)

        for (dependency in command.dependencies) {
            if (dependency !in visited) {
                topologicalSort(dependency, visited, stack)
            }
        }

        stack.push(command)
    }

    /**
     * Sorts and executes all commands.
     *
     * This should be the only method called in the main loop.
     *
     * @see Command
     */
    fun update() {
        val startTime = java.lang.System.currentTimeMillis()

        addQueue.forEach(this::internalAdd)
        addQueue.clear()

        if (changed) {
            sort()
            changed = false
        }

        sortedCommands.forEach(Command<*>::execute)
        val longestTime = sortedCommands.maxBy { it.state.executionTime.seconds() }

        removeQueue.forEach(this::internalRemove)
        removeQueue.clear()

        loopDeltaTime = Time.fromMilliseconds(java.lang.System.currentTimeMillis() - startTime)
        println("The longest time was ${longestTime.state.name} at ${longestTime.state.executionTime.milliseconds()}ms taking ${(longestTime.state.executionTime / loopDeltaTime).seconds() * 100}%")
    }

    fun clear() {
        commands.clear()
        sortedCommands.clear()
        addQueue.clear()
        removeQueue.clear()
    }
}