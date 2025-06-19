package org.firstinspires.ftc.teamcode.input

enum class Gamepads {
    GAMEPAD_1,
    GAMEPAD_2,
    BOTH
}

fun Gamepads.matches(gamepad: Gamepads): Boolean = this == gamepad || this == Gamepads.BOTH