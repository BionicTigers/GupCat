package io.github.bionictigers.input

class InputGroup {
    val buttonChecks = mutableListOf<BaseButton<*>>()
    val lambdaChecks = mutableListOf<() -> Boolean>()

    fun isDown(button: BaseButton<*>): InputGroup {
        buttonChecks.add(button)
        return this
    }

    fun isUp(button: BaseButton<*>): InputGroup {
        buttonChecks.add(button)
        return this
    }

    fun check(lambda: () -> Boolean): InputGroup {
        lambdaChecks.add(lambda)
        return this
    }

    fun build(): InputGroupCommand {
        return TODO("Not yet implemented")
    }
}

class InputGroupCommand {}