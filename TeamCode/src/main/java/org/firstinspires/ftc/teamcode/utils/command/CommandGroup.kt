package org.firstinspires.ftc.teamcode.utils.command

class CommandGroup {
    private val commandList: ArrayList<() -> Command> = arrayListOf()
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

            if (active != null && !active!!.context.inScheduler) {
                commandList.removeAt(0)
                println("Moving to ${commandList.size}")
                if (commandList.isNotEmpty()) {
                    active = null
                }
            }

            if (active == null) {
                val command = commandList[0]()
                active = command
                Scheduler.addToQueue(command)
            }
        }) { commandList.isNotEmpty() }
    }
}