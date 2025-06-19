package io.github.bionictigers.axiom.commands

import com.qualcomm.robotcore.util.RobotLog
import io.github.bionictigers.axiom.web.Editable
import io.github.bionictigers.axiom.web.Hidden
import io.github.bionictigers.axiom.web.ObjectType
import io.github.bionictigers.axiom.web.Value
import io.github.bionictigers.io.github.bionictigers.axiom.utils.convertTo
import io.github.bionictigers.io.github.bionictigers.axiom.utils.hasAnnotationOnProperty
import io.github.bionictigers.io.github.bionictigers.axiom.web.Display
import java.lang.reflect.Field
import java.util.*
import java.util.concurrent.ConcurrentHashMap
import kotlin.collections.ArrayList
import kotlin.collections.HashMap
import kotlin.time.TimeSource

typealias GenericCommand = Command<out BaseCommandState>

object Scheduler {
    private val commands = ConcurrentHashMap<Int, GenericCommand>()
    private val sortedCommands = ArrayList<GenericCommand>()

    private val systems = ConcurrentHashMap<Int, System>()
//    private val systemsToCommands = ConcurrentHashMap<Int, Int>()

    private val addQueue: ArrayList<GenericCommand> = ArrayList()
    private val removeQueue: ArrayList<GenericCommand> = ArrayList()
    private val editQueue: ArrayList<Pair<String, Any>> = ArrayList()

    private val persistentStates = ConcurrentHashMap<String, BaseCommandState>()

    private var changed = false
    private var inUpdateCycle = false

//    init {
//        Server.start()
//    }

    /**
     * Adds commands to the scheduler.
     *
     * Duplicate commands will be ignored.
     *
     * @param command The commands to add.
     * @see Command
     */
    fun add(vararg command: GenericCommand) {
        if (inUpdateCycle)
            addQueue.addAll(command)
        else {
            command.forEach {
                internalAdd(it)
            }
        }
    }

    fun add(commands: Collection<GenericCommand>) {
        commands.forEach {
            add(it)
        }
    }

    private fun serializeVariable(state: Any?, readOnly: Boolean): Any {
        return when (state) {
            is Number, is String, is Char, is Boolean -> Value(state, readOnly)
            is Collection<*> -> state.map { return@map serializeVariable(it!!, false) }
            else -> {
                if (state == null) return Value(null, readOnly)
                val map = hashMapOf<String, Any?>()
                state::class.java.declaredFields.forEach { field ->
                    if (hasAnnotationOnProperty<Display>(state, field.name)) {
                        field.isAccessible = true
                        map[field.name] = serializeVariable(field.get(state), readOnly || !hasAnnotationOnProperty<Editable>(state, field.name))
                    }
                }
                return map.ifEmpty { throw InternalError("Failed to serialize") }
            }
        }
    }

    private fun serializeState(cmdState: Any): Map<String, Any>? {
        val map = HashMap<String, Any>()

        (cmdState::class.java.declaredFields + cmdState::class.java.superclass.declaredFields).forEach {
            val isHidden = hasAnnotationOnProperty<Hidden>(cmdState, it.name)
            val isEditable = hasAnnotationOnProperty<Editable>(cmdState, it.name)
            if (it.isSynthetic || it.name == "name" || isHidden) return@forEach
            it.isAccessible = true
            try {
                val value = it.get(cmdState)?.let { it1 -> serializeVariable(it1, !isEditable) }
                if (value != null) map[it.name] = value
            } catch(_: InternalError) {
//                println("Failed to serialize ${it.name} from ${cmdState.name} is ${it.type}")
            }
        }

        return map.ifEmpty { null }
    }

    fun serialize(): ArrayList<Map<String, Any>> {
        val array = ArrayList<Map<String, Any>>()
        commands.values.forEach {
            array += mapOf("name" to it.state.name, "hash" to it.hashCode(), "state" to (serializeState(it.state) ?: mapOf()), "type" to ObjectType.Command)
        }
        systems.values.forEach {
            array += mapOf("name" to it.name, "hash" to it.hashCode(), "state" to (serializeState(it) ?: mapOf()), "type" to ObjectType.System)
        }

        return array
    }

    private fun internalAdd(command: GenericCommand) {
        changed = true
        commands[command.hashCode()] = command
        command.enter()
    }

    fun <T: BaseCommandState> getPersistentState(name: String, default: T, onGet: T.() -> Unit = {}): T {
        @Suppress("UNCHECKED_CAST")
        //Default is unreachable but required to satisfy the compiler
        if (persistentStates.containsKey(name) && persistentStates[name] as? T != null) {
            val state = persistentStates[name] as? T ?: default
            onGet(state)
            persistentStates[name] = state
            return state
        }

        persistentStates[name] = default
        return default
    }

    /**
     * Adds systems to the scheduler.
     *
     * This is a convenience method that adds the beforeRun and afterRun commands of the system.
     *
     * @param system The systems to add.
     * @see System
     */
    fun addSystem(vararg system: System) {
        add(system.mapNotNull { it.beforeRun })
        add(system.mapNotNull { it.afterRun })
        system.forEach {
            systems[it.hashCode()] = it
        }
    }

    /**
     * Removes a command from the scheduler.
     *
     * If the command is not in the scheduler, it will be ignored.
     *
     * @param command The command to remove.
     * @see Command
     */
    fun remove(vararg command: GenericCommand) {
        command.forEach {
            if (it !in commands.values) {
                return
            }

            removeQueue.add(it)
        }
    }

    private fun internalRemove(command: GenericCommand) {
        changed = true
        commands.remove(command.hashCode())
        command.dependencies.forEach { dep ->
            if (dep !in commands.values) {
                return
            }
            dep.dependencies.remove(command)
        }
        command.exit()
        command.reset()
    }

    private fun sort() {
        val visited = HashSet<GenericCommand>()
        val stack = Stack<GenericCommand>()

        for (command in commands.values) {
            if (command !in visited) {
                topologicalSort(command, visited, stack)
            }
        }

        sortedCommands.clear()
        while (stack.isNotEmpty()) {
            sortedCommands.add(stack.pop())
        }
    }

    private fun topologicalSort(command: GenericCommand, visited: HashSet<GenericCommand>, stack: Stack<GenericCommand>) {
        visited.add(command)

        for (dependency in command.dependencies) {
            if (dependency !in visited) {
                topologicalSort(dependency, visited, stack)
            }
        }

        stack.push(command)
    }

    fun edit(path: String, value: Any) {
        if (inUpdateCycle)
            editQueue.add(path to value)
        else
            internalEdit(path to value)
    }

    //TODO: Make more universal/delegate
    private fun internalEdit(edit: Pair<String, Any>) {
        val (path, value) = edit
        val splitPath = path.split(".")
        val type = if (splitPath[0] == "Command") commands else systems
        val command = type[splitPath[1].toInt()] ?: return

        //Unsafe magic!
        var index = 2
        try {
            var obj: Any = if (splitPath[0] == "Command") (command as GenericCommand).state else command
            lateinit var field: Field
            splitPath.subList(2, splitPath.size).forEach { fieldName ->
                field = obj::class.java.getDeclaredField(fieldName)
                field.isAccessible = true
                index++
                if (splitPath.size != index) obj = field.get(obj)!!
            }

            val target = field.get(obj)!!

            field.set(obj, (value as String).convertTo(target::class))
        } catch(e: NoSuchFieldException) {
            RobotLog.ww("Axiom", "Unable to modify $path, failed to find $index: ${splitPath[index]} in ${splitPath[index - 1]}.")
        } catch(e: NumberFormatException) {
            RobotLog.ww("Axiom", "Unable to set $path to $value as they do not share a type.")
        } catch(e: IllegalArgumentException) {
            RobotLog.ww("Axiom", "Unable to set $path to $value as they do not share a type.")
        }
    }

    /**
     * Sorts and executes all commands.
     *
     * This should be the only method called in the main loop.
     *
     * @see Command
     */
    fun update() {
        inUpdateCycle = true
        val startTime = java.lang.System.currentTimeMillis()

        editQueue.forEach(this::internalEdit)
        editQueue.clear()

        addQueue.forEach(this::internalAdd)
        addQueue.clear()

        if (changed) {
            sort()
            changed = false
        }

        sortedCommands.forEach(GenericCommand::execute)

        removeQueue.forEach(this::internalRemove)
        removeQueue.clear()

        loopDeltaTime = (java.lang.System.currentTimeMillis() - startTime).milliseconds

        inUpdateCycle = false
    }

    fun clear() {
        commands.clear()
        sortedCommands.clear()
        addQueue.clear()
        removeQueue.clear()
    }
}