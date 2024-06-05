package atk.commands

import atk.utils.Time
import java.util.UUID

data class CommandContext internal constructor(
    val id: UUID = UUID.randomUUID(),
    var startTime: Time = Time(),
    var deltaTime: Time = Time(),
    var elapsedTime: Time = Time(),
    var inScheduler: Boolean = false,
    val netVariables: HashMap<String, Any> = hashMapOf(),
)

open class Command(
    val callback: (CommandContext) -> Unit,
    val predicate: (CommandContext) -> Boolean = { false },
) {
    internal val context = CommandContext()

    fun execute() {
        callback(context)
        if (!predicate(context))
            remove()
    }

    fun remove() {
        assert(context.inScheduler) { "Command must be initialized before removing it" }
        Scheduler.remove(context.id)
    }

    override fun toString(): String {
        return "${context.id}, ${context.deltaTime}, ${context.elapsedTime}"
    }
}