package io.github.bionictigers.input

import org.firstinspires.ftc.teamcode.axiom.commands.Command
import org.firstinspires.ftc.teamcode.axiom.commands.CommandState
import org.firstinspires.ftc.teamcode.axiom.commands.Scheduler
import org.firstinspires.ftc.teamcode.axiom.commands.statelessCommand
import org.firstinspires.ftc.teamcode.axiom.input.BaseButton
import org.firstinspires.ftc.teamcode.axiom.input.Gamepad

class BooleanButton(value: Boolean) : BaseButton<Boolean>(value) {
    val onDown = ArrayList<() -> Unit>()
    val onUp = ArrayList<() -> Unit>()
    val onHold = ArrayList<() -> Unit>()

    fun onDown(lambda: () -> Unit): BooleanButton {
        onDown.add(lambda)
        return this
    }

    fun onUp(lambda: () -> Unit): BooleanButton {
        onUp.add(lambda)
        return this
    }

    override fun update(value: Boolean) {
        super.update(value)

        if (lastValue != value && value == true) {
            Scheduler.add(onDown.map {
                statelessCommand("BooleanButton OnDown")
                    .dependsOn(command as Command<CommandState>)
                TODO("FIX REQUIREMENT OF COMMANDSTATE GENERIC")
            })
        }
    }
}