package io.github.bionictigers.axiom.commands.groups

import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.InstantCommand
import io.github.bionictigers.axiom.commands.WaitCommand

@DslMarker
annotation class CommandGroupDsl

@CommandGroupDsl
interface CommandGroupBuilder {
    fun add(command: Command<*>)
    fun run(name: String? = null, block: (BaseCommandState) -> Unit)
    fun continuous(name: String? = null, block: (BaseCommandState) -> Unit)
    fun wait(duration: Time, name: String? = null)
}

internal class CommandGroupBuilderImpl : CommandGroupBuilder {
    val commands = mutableListOf<Command<*>>()

    override fun add(command: Command<*>) {
        commands.add(command)
    }

    override fun run(name: String?, block: (BaseCommandState) -> Unit) {
        commands.add(InstantCommand(name ?: "Instant Command", actionToRun = block))
    }

    override fun continuous(name: String?, block: (BaseCommandState) -> Unit) {
        add(Command.continuous(name ?: "Continuous Command", action = block))
    }

    override fun wait(duration: Time, name: String?) {
        commands.add(WaitCommand(name ?: "Wait Command", duration = duration))
    }
}