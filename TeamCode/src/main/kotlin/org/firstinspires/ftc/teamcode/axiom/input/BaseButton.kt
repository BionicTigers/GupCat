package io.github.bionictigers.input

open class BaseButton<T>(var value: T) {
    open fun update(value: T) {
        this.value = value
    }
}