package org.firstinspires.ftc.teamcode.axiom.commands

import io.github.bionictigers.commands.System
import org.firstinspires.ftc.teamcode.utils.Time

interface CommandState {
    val name: String
    var enteredAt: Long
    var timeInScheduler: Long
    var lastExecutedAt: Long
    var deltaTime: Time

    companion object {
        fun default(name: String = "Unnamed Command"): CommandState {
            return object : CommandState {
                override val name: String = name
                override var enteredAt: Long = 0
                override var timeInScheduler: Long = 0
                override var lastExecutedAt: Long = 0
                override var deltaTime: Time = Time.fromSeconds(0.0)
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
class Command<T: CommandState>(val state: T) {
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
    fun dependsOn(systems: List<System>): Command<T> {
        dependencies.addAll(systems.mapNotNull { it.beforeRun })
        return this
    }

    /**
     * Adds commands that the command depends on.
     *
     * @param commands The commands that the command depends on.
     * @see Command
     */
    fun dependsOn(vararg commands: Command<T>): Command<T> {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Adds a list of commands that the command depends on.
     *
     * @param commands The list of commands that the command depends on.
     * @see Command
     */
    fun dependsOn(commands: List<Command<T>>): Command<T> {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Assigns a function to be invoked during command execution.
     *
     * @param lambda The function to be invoked. The value returned in the lambda determines if the command stays in the scheduler.
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
        val currentTime = java.lang.System.currentTimeMillis()

        if (state.enteredAt == 0L)
            state.enteredAt = java.lang.System.currentTimeMillis()

        state.deltaTime = Time.fromSeconds((currentTime - state.lastExecutedAt) / 1000.0)
        state.timeInScheduler = currentTime - state.enteredAt
        state.lastExecutedAt = state.timeInScheduler

        var result = false

        if (!running) {
            onEnter(state)
            running = true
        }

        if (predicate(state)) {
            result = action(state)
        }

        if (result) { // why isnt this !result
            onExit(state)
            running = false
        }

        return result
    }
}

fun statelessCommand(name: String = "Unnamed Command"): Command<CommandState> {
    return Command(CommandState.default(name))
}