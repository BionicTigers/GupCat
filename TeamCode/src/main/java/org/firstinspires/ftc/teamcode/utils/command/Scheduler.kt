package org.firstinspires.ftc.teamcode.utils.command

import org.firstinspires.ftc.teamcode.utils.Time
import java.util.UUID

object Scheduler {
    private val commandList: HashMap<UUID, Command> = hashMapOf()
    private val orderedList: HashMap<Int, UUID> = hashMapOf()
    private val unorderedList: HashMap<Int, UUID> = hashMapOf()

    private val addQueue: ArrayList<Command> = arrayListOf()

    private var totalAdded = 0

    private var loopStartTime = System.currentTimeMillis()
    var deltaTime = Time.fromSeconds(0.0)

    private fun refreshDeltaTime() {
        deltaTime = Time.fromMilliseconds((System.currentTimeMillis() - loopStartTime).toDouble())
        loopStartTime = System.currentTimeMillis()
    }

    private fun setupInternals(command: Command) {
        command.context.inScheduler = true
        command.context.startTime = Time.fromMilliseconds(System.currentTimeMillis().toDouble())
        commandList[command.context.id] = command
    }

    fun add(command: Command): CommandContext {
        commandList[command.context.id] = command
        unorderedList[totalAdded] = command.context.id
        setupInternals(command)
        totalAdded += 1

        return command.context
    }

    fun set(command: Command, index: Int): CommandContext {
        commandList[command.context.id] = command
        orderedList[index] = command.context.id
        setupInternals(command)

        return command.context
    }

    internal fun addToQueue(command: Command) {
        addQueue.add(command)
        totalAdded += 1
    }

    fun contains(id: UUID): Boolean {
        return commandList.contains(id)
    }

    fun remove(context: CommandContext) {
        remove(context.id)
    }

    fun remove(id: UUID) {
        val command = commandList[id]
        if (command != null && command.context.inScheduler) {
            command.context.inScheduler = false
            commandList.remove(id)
        }
    }

    private fun updateContext(command: Command) {
        command.context.elapsedTime += deltaTime
    }

    private fun executeCommand(command: Command) {
        updateContext(command)
        command.execute()
    }

    fun update() {
        refreshDeltaTime()
        val orderedRemoveQueue = arrayListOf<Int>()
        val unorderedRemoveQueue = arrayListOf<Int>()

        orderedList.forEach { (index, id) ->
            val command = commandList[id]
            if (command != null)
                executeCommand(command)
            else
                orderedRemoveQueue.add(index)
        }

        orderedRemoveQueue.forEach {
            orderedList.remove(it)
        }

        unorderedList.forEach { (index, id) ->
            val command = commandList[id]
            if (command != null)
                executeCommand(command)
            else
                unorderedRemoveQueue.add(index)
        }

        unorderedRemoveQueue.forEach {
            unorderedList.remove(it)
        }

        addQueue.forEach {
            setupInternals(it)
            commandList[it.context.id] = it
            unorderedList[totalAdded] = it.context.id
        }

        addQueue.clear()
    }

    fun clear() {
        commandList.clear()
        unorderedList.clear()
        orderedList.clear()

        update()
    }
}