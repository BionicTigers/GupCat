package io.github.bionictigers.axiom.commands

data class Value(val value: Any, val readOnly: Boolean = false)

abstract class Display {
    abstract fun serialize(): Map<String, Value>
}