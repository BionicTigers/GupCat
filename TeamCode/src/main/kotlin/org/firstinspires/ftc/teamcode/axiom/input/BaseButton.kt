package org.firstinspires.ftc.teamcode.axiom.input

import org.firstinspires.ftc.teamcode.axiom.commands.Command
//import org.firstinspires.ftc.teamcode.axiom.web.Display

enum class BaseButtonState {
    None
}

open class BaseButton<T>(value: T) /**: Display */ {
    var value = value
        private set

    internal var lastValue: T = value
        private set

    open fun update(value: T): List<((T) -> Unit)> {
        this.lastValue = this.value
        this.value = value

        return emptyList()
    }

//    override fun Serialize(): Map<Any, Any> {
//        TODO("Not yet implemented")
//    }
}