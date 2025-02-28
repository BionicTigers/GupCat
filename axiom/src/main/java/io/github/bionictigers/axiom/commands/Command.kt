package io.github.bionictigers.axiom.commands

import io.github.bionictigers.axiom.utils.Time

interface CommandState {
    val name: String
    var enteredAt: Time
    var timeInScheduler: Time
    var lastExecutedAt: Time
    var deltaTime: Time

    companion object {
        fun default(name: String = "Unnamed Command"): CommandState {
            return object : CommandState {
                override val name = name
                override var enteredAt = Time()
                override var timeInScheduler = Time()
                override var lastExecutedAt = Time()
                override var deltaTime = Time.fromSeconds(0.0)
            }
        }
    }
}

/**
 * Commands are used to execute functions in the scheduler.
 *
 * They are made to be fully extensible and can be used to create complex systems.
 *
 * @see Scheduler
 * @see System
 */
data class Command<T: CommandState>(val state: T, private val interval: Time? = null) {
    val dependencies = ArrayList<Command<*>>()

    private var predicate: (T) -> Boolean = { true }
    private var action: (T) -> Boolean = { false }

    private var onEnter: (T) -> Unit = {}
    private var onExit: (T) -> Unit = {}

    private var running: Boolean = false

    /**
     * Adds systems that the command depends on.
     *
     * @param systems The systems that the command depends on.
     * @see System
     */
    fun dependsOn(vararg systems: System): Command<T> {
        dependencies.addAll(systems.mapNotNull { it.beforeRun })
        return this
    }

    /**
     * Adds a list of systems that the command depends on.
     *
     * @param systems The list of systems that the command depends on.
     * @see System
     */
    fun dependsOnSystem(systems: List<System>): Command<T> {
        dependencies.addAll(systems.mapNotNull { it.beforeRun })
        return this
    }

    /**
     * Adds commands that the command depends on.
     *
     * @param commands The commands that the command depends on.
     * @see Command
     */
    fun dependsOn(vararg commands: Command<*>): Command<T> {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Adds a list of commands that the command depends on.
     *
     * @param commands The list of commands that the command depends on.
     * @see Command
     */
    fun dependsOn(commands: List<Command<*>>): Command<T> {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Assigns a function to be invoked during command execution.
     *
     * @param lambda The function to be invoked. The value returned in the lambda determines if the command stays in the scheduler. True means it leaves the scheduler.
     */
    fun setAction(lambda: (T) -> Boolean): Command<T> {
        action = lambda
        return this
    }

    /**
     * Executes the command if the predicate is true.
     * If the predicate is false, the command will be removed from the scheduler.
     *
     * @param lambda The predicate to be invoked. The value returned in the lambda determines if the command should be executed.
     */
    fun setPredicate(lambda: (T) -> Boolean): Command<T> {
        predicate = lambda
        return this
    }

    /**
     * Executes when the command is entering the scheduler.
     *
     * @param lambda The function to be invoked.
     */
    fun setOnEnter(lambda: (T) -> Unit): Command<T> {
        onEnter = lambda
        return this
    }

    /**
     * Executes when the command is leaving the scheduler.
     *
     * @param lambda The function to be invoked.
     */
    fun setOnExit(lambda: (T) -> Unit): Command<T> {
        onExit = lambda
        return this
    }

    /**
     * Executes the command.
     *
     * @return True if the command was executed, false otherwise.
     */
    internal fun execute(): Boolean {
        val currentTime = Time.fromMilliseconds(java.lang.System.currentTimeMillis())

        if (interval != null && currentTime - state.enteredAt >= interval) return false

        if (state.enteredAt == Time())
            state.enteredAt = currentTime

        state.deltaTime = currentTime - state.lastExecutedAt
        state.timeInScheduler = currentTime - state.enteredAt
        state.lastExecutedAt = currentTime

        var result = false

        if (!running) {
            onEnter(state)
            running = true
        }

        if (predicate(state)) {
            result = action(state)
        }

        if (result) {
            onExit(state)
            running = false
            Scheduler.remove(this)
        }

        return result
    }
}

fun statelessCommand(name: String = "Unnamed Command"): Command<CommandState> {
    return Command(CommandState.default(name))
}