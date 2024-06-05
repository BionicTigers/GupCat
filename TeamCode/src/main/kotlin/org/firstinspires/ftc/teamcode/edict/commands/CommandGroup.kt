package atk.commands

class CommandGroup {
    private val commandList: ArrayList<Command> = arrayListOf()
    private var executed = false

    fun add(command: Command): CommandGroup {
        commandList.add(command)
        return this
    }

    fun build(): Command {
        return Command({
            if (commandList.isEmpty()) return@Command

            if (!commandList[0].context.inScheduler && executed) {
                commandList.removeAt(0)
                if (commandList.isNotEmpty())
                    Scheduler.addToQueue(commandList[0])
            }

            if (!executed) {
                Scheduler.addToQueue(commandList[0])
                executed = true
            }
        }, { commandList.isNotEmpty() })
    }
}