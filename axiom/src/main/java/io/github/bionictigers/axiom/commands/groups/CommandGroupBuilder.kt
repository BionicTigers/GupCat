package io.github.bionictigers.axiom.commands.groups

import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import kotlin.time.Duration

@DslMarker
annotation class CommandGroupDsl

class CommandGroupBuilder {
    val commands = mutableListOf<Command<*>>()

    fun add(command: Command<*>) {
        commands.add(command)
    }

    fun instant(name: String? = null, block: (BaseCommandState) -> Unit) {
        add(Command.instant(name ?: "Instant Command", block))
    }

    fun continuous(name: String? = null, block: (BaseCommandState) -> Unit) {
        add(Command.continuous(name ?: "Continuous Command", action = block))
    }

    fun wait(duration: Duration, name: String? = null) {
        add(Command.wait(name ?: "Wait Command", duration))
    }

    fun waitUntil(name: String? = null, predicate: (BaseCommandState) -> Boolean) {
        add(Command.create(name ?: "Wait Until") {
            requires {
                predicate(it)
            }

            action { true }
        })
    }
}