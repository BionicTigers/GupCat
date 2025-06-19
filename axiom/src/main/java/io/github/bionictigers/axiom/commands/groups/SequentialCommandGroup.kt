package io.github.bionictigers.axiom.commands.groups

import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler

data class SequentialCommandGroupState(
    val commands: List<String>,
    var currentIndex: Int = 0
) : BaseCommandState()

class SequentialCommandGroup(
    name: String = "SequentialCommandGroup",
    val commands: List<Command<*>>
) : Command<SequentialCommandGroupState>(
    name,
    SequentialCommandGroupState(commands.map { it.name })
) {
    private var currentCommand: Command<*>

    init {
        require(commands.isNotEmpty()) { "SequentialCommandGroup must have at least one command." }

        currentCommand = commands.first()

        enter {
            Scheduler.schedule(currentCommand)
        }

        action {
            if (!currentCommand.running && commands.size > it.currentIndex + 1) {
                it.currentIndex++
                currentCommand = commands[it.currentIndex]
                Scheduler.schedule(currentCommand)
            } else if (it.currentIndex == commands.size - 1 && !currentCommand.running) {
                println("Finished all commands in SequentialCommandGroup: $name")
                return@action true
            }

            false
        }
    }
}

fun sequential(name: String = "SequentialCommandGroup", block: CommandGroupBuilder.() -> Unit): SequentialCommandGroup {
    val builder = CommandGroupBuilderImpl()
    builder.block()
    return SequentialCommandGroup(name, builder.commands)
}

@CommandGroupDsl
fun CommandGroupBuilder.sequential(name: String = "SequentialCommandGroup", block: CommandGroupBuilder.() -> Unit) {
    this.add(io.github.bionictigers.axiom.commands.groups.sequential(name, block)) // definition above
}