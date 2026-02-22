package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command

class UninteraptingCommand(val command: Command) : Command() {
    override val isDone: Boolean
        get() = true

    override fun start() {
        command.schedule()
    }
}