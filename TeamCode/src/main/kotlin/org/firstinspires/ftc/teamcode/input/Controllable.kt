package org.firstinspires.ftc.teamcode.input

interface Controllable {
    fun bindControls(profile: Profile, gamepad: Gamepads, builder: Controls.Builder)
}