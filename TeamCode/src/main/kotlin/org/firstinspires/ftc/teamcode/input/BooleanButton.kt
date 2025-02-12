package org.firstinspires.ftc.teamcode.input

class BooleanButton(value: Boolean) : BaseButton<Boolean>(value) {
    enum class ButtonState {
        UP, DOWN, HOLD, NONE
    }

    var state = ButtonState.NONE
        private set

    private val onDown = ArrayList<(Boolean) -> Unit>()
    private val onUp = ArrayList<(Boolean) -> Unit>()
    private val onHold = ArrayList<(Boolean) -> Unit>()

    fun onDown(lambda: (Boolean) -> Unit): BooleanButton {
        onDown.add(lambda)
        return this
    }

    fun onUp(lambda: (Boolean) -> Unit): BooleanButton {
        onUp.add(lambda)
        return this
    }

    fun onHold(lambda: (Boolean) -> Unit): BooleanButton {
        onHold.add(lambda)
        return this
    }

    override fun update(value: Boolean): List<(Boolean) -> Unit> {
        state = when {
            value && !this.value -> ButtonState.DOWN
            !value && this.value -> ButtonState.UP
            value -> ButtonState.HOLD
            else -> ButtonState.NONE
        }

        val lambdaList = when (state) {
            ButtonState.DOWN -> onDown
            ButtonState.UP -> onUp
            ButtonState.HOLD -> onHold
            else -> emptyList()
        }

        super.update(value)

        return lambdaList
    }
}