package org.firstinspires.ftc.teamcode.utils.command

class CommandGroup {
    private val commandList: ArrayList<() -> Command> = arrayListOf()
    private var executed = false
    private var active: Command? = null

    fun add(command: Command): CommandGroup {
        commandList.add { command }
        return this
    }

    fun add(command: () -> Command): CommandGroup {
        commandList.add(command)
        return this
    }

    fun build(): Command {
        return Command({
            if (commandList.isEmpty()) return@Command

            if (active != null && !active!!.context.inScheduler && executed) {
                commandList.removeAt(0)
                println("Moving to ${commandList.size}")
                if (commandList.isNotEmpty()) {
                    val command = commandList[0]()
                    active = command
                    Scheduler.addToQueue(command)
                }
            }

            if (!executed) {
                val command = commandList[0]()
                active = command
                Scheduler.addToQueue(command)
                executed = true
            }
        }) { commandList.isNotEmpty() }
    }
}