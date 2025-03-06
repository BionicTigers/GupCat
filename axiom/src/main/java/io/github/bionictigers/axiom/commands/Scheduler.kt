package io.github.bionictigers.axiom.commands

import com.qualcomm.robotcore.util.RobotLog
import io.github.bionictigers.axiom.utils.Time
import io.github.bionictigers.axiom.web.Editable
import io.github.bionictigers.axiom.web.Hidden
import io.github.bionictigers.io.github.bionictigers.axiom.utils.convertTo
import io.github.bionictigers.io.github.bionictigers.axiom.utils.hasAnnotationOnProperty
import io.github.bionictigers.io.github.bionictigers.axiom.web.Display
import io.github.bionictigers.axiom.web.Value
import java.lang.reflect.Field
import java.util.*
import java.util.concurrent.ConcurrentHashMap
import kotlin.collections.ArrayList
import kotlin.collections.HashMap

object Scheduler {
    private val commands = ConcurrentHashMap<Int, Command<*>>()
    private val sortedCommands = ArrayList<Command<*>>()

    private val addQueue: ArrayList<Command<*>> = ArrayList()
    private val removeQueue: ArrayList<Command<*>> = ArrayList()
    private val editQueue: ArrayList<Pair<String, Any>> = ArrayList()

    private var changed = false
    private var inUpdateCycle = false

    var loopDeltaTime = Time()

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
    fun add(vararg command: Command<*>) {
        if (inUpdateCycle)
            addQueue.addAll(command)
        else {
            command.forEach {
                internalAdd(it)
            }
        }
    }

    fun add(commands: Collection<Command<*>>) {
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

    private fun serializeState(cmdState: CommandState): Map<String, Any> {
        val map = HashMap<String, Any>()

        cmdState::class.java.declaredFields.forEach {
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

        map["deltaTime"] = 0.0

        return map
    }

    fun serialize(): ArrayList<Map<String, Any>> {
        val array = ArrayList<Map<String, Any>>()
        commands.values.forEach {
            array.add(mapOf("name" to it.state.name, "hash" to it.hashCode(), "state" to serializeState(it.state)))
        }

        return array
    }

    private fun internalAdd(command: Command<*>) {
        changed = true
        commands[command.hashCode()] = command
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
    }

    /**
     * Removes a command from the scheduler.
     *
     * If the command is not in the scheduler, it will be ignored.
     *
     * @param command The command to remove.
     * @see Command
     */
    fun remove(vararg command: Command<*>) {
        command.forEach {
            if (it !in commands.values) {
                return
            }

            removeQueue.add(it)
        }
    }

    private fun internalRemove(command: Command<*>) {
        changed = true
        commands.remove(command.hashCode())
    }

    private fun sort() {
        val visited = HashSet<Command<*>>()
        val stack = Stack<Command<*>>()

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

    private fun topologicalSort(command: Command<*>, visited: HashSet<Command<*>>, stack: Stack<Command<*>>) {
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

    private fun internalEdit(edit: Pair<String, Any>) {
        val (path, value) = edit
        val splitPath = path.split(".")
        val command = commands[splitPath[0].toInt()] ?: return

        //Unsafe magic!
        var index = 1
        try {
            var obj: Any = command.state
            lateinit var field: Field
            splitPath.subList(1, splitPath.size).forEach { fieldName ->
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

        sortedCommands.forEach(Command<*>::execute)

        removeQueue.forEach(this::internalRemove)
        removeQueue.clear()

        loopDeltaTime = Time.fromMilliseconds(java.lang.System.currentTimeMillis() - startTime)

        inUpdateCycle = false
    }

    fun clear() {
        commands.clear()
        sortedCommands.clear()
        addQueue.clear()
        removeQueue.clear()
    }
}