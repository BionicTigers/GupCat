package org.firstinspires.ftc.teamcode.axiom.input

import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.web.Display

open class BaseButton<T>(value: T, internal val command: Command<*>) : Display {
    var value = value
        private set
    internal var lastValue: T = value
        private set

    open fun update(value: T) {
        this.lastValue = this.value
        this.value = value
    }

    override fun Serialize(): Map<Any, Any> {
        TODO("Not yet implemented")
    }
}