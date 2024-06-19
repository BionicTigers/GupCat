package io.github.bionictigers.commands

/**
 * A system is the base class for all systems/mechanisms.
 * Examples include Gamepad, Drivetrain, and Intake.
 *
 * All commands should be associated with at least one system.
 *
 * @property priority The priority of the system. The higher the number, the higher the priority. This is used to sort commands in the scheduler.
 * @see Scheduler
 * @see Command
 */
interface System {
    val dependencies: List<System>

    val beforeRun: Command?
    val afterRun: Command?
}