package org.firstinspires.ftc.teamcode.utils.command

import com.qualcomm.robotcore.util.ElapsedTime
import java.sql.Time
import java.time.Duration
import java.util.concurrent.TimeUnit

/**
 * Executes code in a linear manner stepping through whenever the previous command is finished.
 *
 */
@Suppress("LABEL_NAME_CLASH")
class CommandGroup {
    private val callbacks: ArrayList<() -> Boolean> = ArrayList()

    /**
     * Add a OnceCommand to the command group.
     */
    fun add(command: OnceCommand): CommandGroup {
        callbacks.add { command.callback.invoke(); return@add false }
        return this
    }

    /**
     * Add a ConditionalCommand to the command group.
     */
    fun add(command: ConditionalCommand?): CommandGroup {
        if (command != null) {
            callbacks.add { command.callback.invoke(); return@add command.predicate.invoke() }
        }
        return this
    }

    /**
     * Func
     */
//    fun add(command: ConditionalCommand?): CommandGroup {
//        if (command != null) {
//            callbacks.add { command.callback.invoke(); return@add command.predicate.invoke() }
//        }
//        return this
//    }

    /**
     * Removes a callback
     */
//    fun remove(command: Int) {
//
//    }

    /**
     * Add a callback to the command group.
     *
     * The return boolean is if it should stay in the group.
     */
//    fun add(callback: () -> Boolean): CommandGroup {
//        callbacks.add(callback)
//        return this
//    }

    /**
     * Wait for another command to finish running.
     */
    fun await(command: Command?): CommandGroup {
        //Check if the command has a priority, if it doesn't then it's not in the scheduler
        if (command != null) {
            callbacks.add {
                return@add command.priority != null
            }
        }
        return this
    }

    /**
     * Wait for time to finish running.
     *
     * @param time Time in Milliseconds
     */
    fun await(time: Int): CommandGroup {
        //Check if the command has a priority, if it doesn't then it's not in the scheduler
        val timer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
        callbacks.add { return@add timer.time() >= time }
        return this
    }

    /**
     * Builds the command into a ConditionalCommand which can be added to the scheduler.
     */
    fun build(): ConditionalCommand {
        return ConditionalCommand(
            {
                val result = callbacks[0].invoke()
                if (!result) {
                    callbacks.removeAt(0)
                }
            },
            {return@ConditionalCommand callbacks.isNotEmpty() }
        )
    }
}