package io.github.bionictigers.axiom.commands.groups

import io.github.bionictigers.axiom.commands.BaseCommandState
import io.github.bionictigers.axiom.commands.Command
import io.github.bionictigers.axiom.commands.Scheduler

enum class ConcurrentFinishMode {
    ALL,
    ANY
}

data class ConcurrentCommandGroupState(
    val commands: List<String>,
    val mode: ConcurrentFinishMode,
) : BaseCommandState()

class ConcurrentCommandGroup(
    name: String = "SequentialCommandGroup",
    val commands: List<Command<*>>,
    private val mode: ConcurrentFinishMode = ConcurrentFinishMode.ALL,
) : Command<ConcurrentCommandGroupState>(
    name,
    ConcurrentCommandGroupState(commands.map { it.name }, mode)
) {
    init {
        require(commands.isNotEmpty()) { "ConcurrentCommandGroup must have at least one command." }

        enter {
            commands.forEach(Scheduler::schedule)
        }

        action { state ->
            val allFinished = commands.all { !it.running }
            val anyFinished = commands.any { !it.running }

            if (mode == ConcurrentFinishMode.ALL && allFinished) {
                println("Finished all commands in ConcurrentCommandGroup: $name")
                return@action true
            } else if (mode == ConcurrentFinishMode.ANY && anyFinished) {
                println("Finished at least one command in ConcurrentCommandGroup: $name")
                return@action true
            }

            false
        }
    }
}

fun concurrent(
    name: String = "ConcurrentCommandGroup",
    mode: ConcurrentFinishMode = ConcurrentFinishMode.ALL,
    block: CommandGroupBuilder.() -> Unit
): ConcurrentCommandGroup {
    val builder = CommandGroupBuilder()
    builder.block()
    return ConcurrentCommandGroup(name, builder.commands, mode)
}

@CommandGroupDsl
fun CommandGroupBuilder.concurrent(
    name: String = "ConcurrentCommandGroup",
    mode: ConcurrentFinishMode = ConcurrentFinishMode.ALL,
    block: CommandGroupBuilder.() -> Unit
) {
    this.add(io.github.bionictigers.axiom.commands.groups.concurrent(name, mode, block))
}