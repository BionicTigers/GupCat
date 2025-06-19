package io.github.bionictigers.axiom.commands

import kotlin.time.Duration
import kotlin.time.TimeSource

/**
 * Commands are used to execute functions in the scheduler.
 *
 * They are made to be fully extensible and can be used to create complex systems.
 *
 * @see Scheduler
 * @see System
 */
@Suppress("unused")
open class Command<T: BaseCommandState> internal constructor(
    val name: String = "Unnamed Command",
    val state: T,
    private val interval: Duration? = null
) {
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
    fun requires(lambda: (T) -> Boolean): Command<T> {
        predicate = lambda
        return this
    }

    /**
     * Executes when the command is entering the scheduler.
     *
     * @param lambda The function to be invoked.
     */
    fun enter(lambda: (T) -> Unit): Command<T> {
        onEnter = lambda
        return this
    }

    /**
     * Executes when the command is leaving the scheduler.
     *
     * @param lambda The function to be invoked.
     */
    fun exit(lambda: (T) -> Unit): Command<T> {
        onExit = lambda
        return this
    }

    /**
     * Executes the command.
     *
     * @return True if the command was executed, false otherwise.
     */
    internal fun execute(): Boolean {
        state.deltaTime = state.lastExecutedAt?.elapsedNow() ?: Duration.ZERO
        state.lastExecutedAt = TimeSource.Monotonic.markNow()
        if (interval != null && state.deltaTime < interval) return false

        var result = false
        if (predicate(state)) {
            result = action(state)
        }

        if (result) {
            Scheduler.remove(this)
        }

        return result
    }

    internal fun internalEnter() {
        onEnter(state)
        state.enteredAt = TimeSource.Monotonic.markNow()
        running = true
    }

    internal fun internalExit() {
        onExit(state)
        state.resetTimings()
        running = false
    }

    companion object {
        fun create(name: String = "Unnamed Command", interval: Duration? = null): Command<BaseCommandState> {
            return Command(name, BaseCommandState(), interval)
        }

        fun <T: BaseCommandState> create(state: T, interval: Duration? = null): Command<T> {
            return Command(state = state, interval = interval)
        }
        fun <T: BaseCommandState> create(name: String = "Unnamed Command", state: T, interval: Duration? = null): Command<T> {
            return Command(name, state, interval)
        }

        fun create(name: String = "Unnamed Command", interval: Duration? = null, block: Command<BaseCommandState>.() -> Unit = {}): Command<BaseCommandState> {
            return Command(name, BaseCommandState(), interval).apply(block)
        }

        fun <T: BaseCommandState> create(state: T, interval: Duration? = null, block: Command<T>.() -> Unit = {}): Command<T> {
            return Command(state = state, interval = interval).apply(block)
        }

        fun <T: BaseCommandState> create(name: String = "Unnamed Command", state: T, interval: Duration? = null, block: Command<T>.() -> Unit = {}): Command<T> {
            return Command(name, state, interval).apply(block)
        }

        fun continuous(name: String = "Continuous Command", interval: Duration? = null, action: (BaseCommandState) -> Unit): Command<BaseCommandState> {
            return Command(name, BaseCommandState(), interval).action { action(it); false }
        }

        fun <T: BaseCommandState> continuous(state: T, interval: Duration? = null, action: (T) -> Unit): Command<T> {
            return Command(state = state, interval = interval).action { action(it); false }
        }

        fun <T: BaseCommandState> continuous(name: String = "Continuous Command", state: T, interval: Duration? = null, action: (T) -> Unit): Command<T> {
            return Command(name, state, interval).action { action(it); false }
        }
    }
}

class WaitCommand(name: String = "Wait Command", private val duration: Duration) : Command<BaseCommandState>(name, BaseCommandState(), duration) {
    init { action { it.enteredAt!!.elapsedNow() >= duration } }
}

class InstantCommand(name: String = "Instant Command", private val actionToRun: (BaseCommandState) -> Unit) : Command<BaseCommandState>(name, BaseCommandState()) {
    init { action { actionToRun(it); true } }
}