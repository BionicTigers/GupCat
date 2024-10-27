package org.firstinspires.ftc.teamcode.axiom.input

import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState

interface InputGroupState : CommandState {
    var state: Boolean

    companion object {
        fun default(): InputGroupState {
            return object : InputGroupState, CommandState by CommandState.default("Input Group") {
                override var state = false
            }
        }
    }
}

class InputGroup {
    private val checks = mutableListOf<() -> Boolean>()

    val onTrue = mutableListOf<() -> Unit>()
    val onFalse = mutableListOf<() -> Unit>()
    val onChange = mutableListOf<(Boolean) -> Unit>()

    fun check(lambda: () -> Boolean): InputGroup {
        checks.add(lambda)
        return this
    }

    fun isDown(button: BaseButton<*>): InputGroup {
        check { button.value == true }
        return this
    }

    fun isUp(button: BaseButton<*>): InputGroup {
        check { button.value == false }
        return this
    }

    fun onTrue(lambda: () -> Unit): InputGroup {
        onTrue.add(lambda)
        return this
    }

    fun onFalse(lambda: () -> Unit): InputGroup {
        onFalse.add(lambda)
        return this
    }

    fun onChange(lambda: (state: Boolean) -> Unit): InputGroup {
        onChange.add(lambda)
        return this
    }

    fun build(): Command<InputGroupState> {
        return Command(InputGroupState.default())
            .setAction {
                val newState = checks.all { it() }

                if (newState != it.state) {
                    it.state = newState
                    onChange.forEach { it(newState ) }
                }

                if (newState)
                    onTrue.forEach { it() }
                else
                    onFalse.forEach { it() }

                false
            }
    }
}
