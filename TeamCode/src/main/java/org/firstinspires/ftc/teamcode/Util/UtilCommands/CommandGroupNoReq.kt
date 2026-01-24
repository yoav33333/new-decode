package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.EmptyGroupException
import kotlin.collections.addAll

abstract class CommandGroupNoReq(vararg val commands: Command) : Command() {

    /**
     * The collection of all commands within this group.
     */
    val children: ArrayDeque<Command> = ArrayDeque(commands.toList())

    init {
        if (commands.isEmpty()) throw EmptyGroupException()
    }

    override fun stop(interrupted: Boolean) {
        children.clear()
        children.addAll(commands)
    }
}