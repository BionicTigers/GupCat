package io.github.bionictigers.commands

/**
 * Commands are used to execute functions in the scheduler.
 *
 * They are made to be fully extensible and can be used to create complex systems.
 *
 * @see Scheduler
 * @see System
 */
class Command {
    internal val dependencies = ArrayList<Command>()

    private var predicate: () -> Boolean = { true }
    private var action: () -> Boolean = { false }

    private var onEnter: () -> Unit = {}
    private var onExit: () -> Unit = {}

    private var running: Boolean = false

    /**
     * Adds systems that the command depends on.
     *
     * @param systems The systems that the command depends on.
     * @see System
     */
    fun dependsOn(vararg systems: System): Command {
        dependencies.addAll(systems.mapNotNull { it.beforeRun })
        return this
    }

    /**
     * Adds a list of systems that the command depends on.
     *
     * @param systems The list of systems that the command depends on.
     * @see System
     */
    fun dependsOn(systems: List<System>): Command {
        dependencies.addAll(systems.mapNotNull { it.beforeRun })
        return this
    }

    /**
     * Adds commands that the command depends on.
     *
     * @param commands The commands that the command depends on.
     * @see Command
     */
    fun dependsOn(vararg commands: Command): Command {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Adds a list of commands that the command depends on.
     *
     * @param commands The list of commands that the command depends on.
     * @see Command
     */
    fun dependsOn(commands: List<Command>): Command {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Assigns a function to be invoked during command execution.
     *
     * @param lambda The function to be invoked. The value returned in the lambda determines if the command stays in the scheduler.
     */
    fun setAction(lambda: () -> Boolean): Command {
        action = lambda
        return this
    }

    /**
     * Executes the command if the predicate is true.
     * If the predicate is false, the command will be removed from the scheduler.
     *
     * @param lambda The predicate to be invoked. The value returned in the lambda determines if the command should be executed.
     */
    fun setPredicate(lambda: () -> Boolean): Command {
        predicate = lambda
        return this
    }

    /**
     * Executes when the command is entering the scheduler.
     *
     * @param lambda The function to be invoked.
     */
    fun setOnEnter(lambda: () -> Unit): Command {
        onEnter = lambda
        return this
    }

    /**
     * Executes when the command is leaving the scheduler.
     *
     * @param lambda The function to be invoked.
     */
    fun setOnExit(lambda: () -> Unit): Command {
        onExit = lambda
        return this
    }

    /**
     * Executes the command.
     *
     * @return True if the command was executed, false otherwise.
     */
    internal fun execute(): Boolean {
        var result = false

        if (!running) {
            onEnter()
            running = true
        }

        if (predicate()) {
            result = action()
        }

        if (result) {
            onExit()
            running = false
        }

        return result
    }
}