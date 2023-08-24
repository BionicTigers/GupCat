package org.firstinspires.ftc.teamcode.utils.command

/**
 * Executes code in a linear manner stepping through whenever the previous command is finished.
 *
 */
@Suppress("LABEL_NAME_CLASH")
class CommandGroup {
    private val callbacks: ArrayList<() -> Boolean> = ArrayList()

    /**
     * Add a OnceCommand to the command group.
     */
    fun add(command: OnceCommand): CommandGroup {
        callbacks.add { command.callback.invoke(); return@add false }
        return this
    }

    /**
     * Add a ConditionalCommand to the command group.
     */
    fun add(command: ConditionalCommand): CommandGroup {
        callbacks.add { command.callback.invoke(); return@add command.predicate.invoke() }
        return this
    }

    /**
     * Add a callback to the command group.
     *
     * The return boolean is if it should stay in the group.
     */
    fun add(callback: () -> Boolean): CommandGroup {
        callbacks.add(callback)
        return this
    }

    /**
     * Wait for another command to finish running.
     */
    fun await(command: Command): CommandGroup {
        //Check if the command has a priority, if it doesn't then it's not in the scheduler
        callbacks.add { return@add command.priority != null }
        return this
    }

    /**
     * Builds the command into a ConditionalCommand which can be added to the scheduler.
     */
    fun build(): ConditionalCommand {
        return ConditionalCommand(
            {
                val result = callbacks[0].invoke()
                if (!result) {
                    callbacks.removeAt(0)
                }
            },
            {return@ConditionalCommand callbacks.isNotEmpty() }
        )
    }
}