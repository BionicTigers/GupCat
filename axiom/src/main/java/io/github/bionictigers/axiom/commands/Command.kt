package io.github.bionictigers.axiom.commands

import kotlinx.datetime.Clock
import kotlinx.datetime.Instant
import kotlin.time.Duration
import kotlin.time.TimeSource

private data class CommandMetadata(
    val name: String
)

/**
 * Commands are used to execute functions in the scheduler.
 *
 * They are made to be fully extensible and can be used to create complex systems.
 *
 * @see Scheduler
 * @see System
 */
@Suppress("unused")
open class Command<T: BaseCommandState> private constructor(
    name: String = "Unnamed Command",
    val state: T,
    private val interval: Duration? = null
) {
    val metadata = CommandMetadata()
    val dependencies = ArrayList<Command<out BaseCommandState>>()

    private var predicate: (T) -> Boolean = { true }
    private var action: (T) -> Boolean = { false }

    internal var onEnter: (T) -> Unit = {}
    internal var onExit: (T) -> Unit = {}

    internal var running = false

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
    fun dependsOn(vararg commands: Command<BaseCommandState>): Command<T> {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Adds a list of commands that the command depends on.
     *
     * @param commands The list of commands that the command depends on.
     * @see Command
     */
    fun dependsOn(commands: List<Command<BaseCommandState>>): Command<T> {
        dependencies.addAll(commands)
        return this
    }

    /**
     * Assigns a function to be invoked during command execution.
     *
     * @param lambda The function to be invoked. The value returned in the lambda determines if the command stays in the scheduler. True means it leaves the scheduler.
     */
    fun action(lambda: (T) -> Boolean): Command<T> {
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
    fun onEnter(lambda: (T) -> Unit): Command<T> {
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
     * Prepares the command to be added to the scheduler if it has already been in it.
     */
    fun reset() {
        state.resetTimings()
        running = false
    }

    /**
     * Executes the command.
     *
     * @return True if the command was executed, false otherwise.
     */
    internal fun execute(): Boolean {
        state.lastExecutedAt = TimeSource.Monotonic.markNow()
        state.deltaTime = currentTime - state.lastExecutedAt
        state.timeInScheduler = currentTime - state.enteredAt
        if (interval != null && state.deltaTime < interval) return false

        state.lastExecutedAt = currentTime

        var result = false
        if (predicate(state)) {
            result = action(state)
        }

        if (result) {
            Scheduler.remove(this)
        }

        return result
    }

    internal fun enter() {
        onEnter(state)
        state.enteredAt = java.lang.System.currentTimeMillis().milliseconds
        running = true
    }

    internal fun exit() {
        onExit(state)
        running = false
    }

    companion object {
        fun create(name: String = "Unnamed Command", interval: Time? = null): Command<BaseCommandState> {
            return Command(BaseCommandState(name), interval)
        }

        fun <T: BaseCommandState> create(state: T, interval: Time? = null): Command<T> {
            return Command(state, interval)
        }

        fun create(name: String = "Unnamed Command", interval: Time? = null, block: Command<BaseCommandState>.() -> Unit = {}): Command<BaseCommandState> {
            return Command(BaseCommandState(name), interval).apply(block)
        }

        fun <T: BaseCommandState> create(state: T, interval: Time? = null, block: Command<T>.() -> Unit = {}): Command<T> {
            return Command(state, interval).apply(block)
        }

        fun continuous(name: String = "Continuous Command", interval: Time? = null, action: (BaseCommandState) -> Unit): Command<BaseCommandState> {
            return Command(BaseCommandState(name), interval).action { action(it); false }
        }

        fun <T: BaseCommandState> continuous(state: T, interval: Time? = null, action: (T) -> Unit): Command<T> {
            return Command(state, interval).action { action(it); false }
        }
    }
}

class WaitCommand(name: String = "Wait Command", private val duration: Time) : Command<BaseCommandState>(BaseCommandState(name), duration) {
    init { action { it.timeInScheduler >= duration } }
}

class InstantCommand(name: String = "Instant Command", private val actionToRun: (BaseCommandState) -> Unit) : Command<BaseCommandState>(BaseCommandState(name)) {
    init { action { actionToRun(it); true } }
}