package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command

/**
 * A command that wraps a finite command and runs it in a continuous loop.
 *
 * This command will run forever, and each time the wrapped command finishes,
 * it will be immediately restarted. This is useful for actions that need
 * to be repeated, like a "shoot one note" command in a loop.
 *
 * This LoopingCommand itself never finishes and must be interrupted by another
 * command that requires the same subsystem.
 *
 * @param command The command to be executed repeatedly. It should be a command
 *                that has a defined end (i.e., its `isDone` property eventually
 *                becomes true).
 */
class LoopingCommand(private val command: Command) : Command() {

    // This command itself never finishes, it must be interrupted.
    override val isDone: Boolean = false

    init {
        // Inherit the requirements from the wrapped command.
        setRequirements(command.requirements)
    }

    override fun start() {
        // When the LoopingCommand starts, start the inner command for the first time.
        command.start()
    }

    override fun update() {
        // First, update the inner command.
        command.update()

        // After updating, check if the inner command has finished.
        if (command.isDone) {
            // It's finished. Stop it...
            command.stop(false) // 'false' because it was not interrupted.
            // ...and immediately start it again to loop.
            command.start()
        }
    }

    override fun stop(interrupted: Boolean) {
        // When this LoopingCommand is stopped (which will always be an interruption),
        // make sure to stop the wrapped command as well.
        command.stop(interrupted)
    }
}
