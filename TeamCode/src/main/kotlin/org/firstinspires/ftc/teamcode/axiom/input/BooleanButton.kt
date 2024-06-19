package io.github.bionictigers.input

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
        val command

        super.update(value)
    }
}