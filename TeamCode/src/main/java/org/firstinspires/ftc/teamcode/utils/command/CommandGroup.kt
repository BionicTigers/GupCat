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
    fun add(command: OnceCommand) {
        callbacks.add { command.callback.invoke(); return@add false }
    }

    /**
     * Add a ConditionalCommand to the command group.
     */
    fun add(command: ConditionalCommand) {
        callbacks.add { command.callback.invoke(); return@add command.predicate.invoke() }
    }

    /**
     * Add a callback to the command group.
     *
     * The return boolean is if it should stay in the group.
     */
    fun add(callback: () -> Boolean) {
        callbacks.add(callback)
    }

    /**
     * Wait for another command to finish running.
     */
    fun await(command: Command) {
        //Check if the command has a priority, if it doesn't then it's not in the scheduler
        callbacks.add { return@add command.priority != null }
    }

    /**
     * Builds the command into a ConditionalCommand which can be added to the scheduler.
     */
    fun build(): ConditionalCommand {
        return ConditionalCommand(
            {
                callbacks[0].invoke()
            },
            {return@ConditionalCommand callbacks.isNotEmpty() }
        )
    }
}