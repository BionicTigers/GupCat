package org.firstinspires.ftc.teamcode.utils.command

import org.firstinspires.ftc.teamcode.utils.Time
import java.util.UUID

data class CommandContext internal constructor(
    val id: UUID = UUID.randomUUID(),
    var startTime: Time = Time(),
    var deltaTime: Time = Time(),
    var elapsedTime: Time = Time(),
    var inScheduler: Boolean = false
)

open class Command(val callback: (CommandContext) -> Unit, val predicate: (CommandContext) -> Boolean) {
    constructor(callback: (deltaTime: CommandContext) -> Unit) : this(callback, { false })
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

fun continuousCommand(callback: (CommandContext) -> Unit): Command {
    return Command(callback) { true }
}

fun timedCommand(callback: (CommandContext) -> Unit, time: Time): Command {
    return Command(callback) { println("${it.elapsedTime.seconds()} == ${time.seconds()}"); it.elapsedTime <= time }
}

fun timedCommand(callback: (CommandContext) -> Unit, predicate: (CommandContext) -> Boolean, time: Time): Command {
    return Command(callback) { predicate(it) || it.elapsedTime >= time }
}