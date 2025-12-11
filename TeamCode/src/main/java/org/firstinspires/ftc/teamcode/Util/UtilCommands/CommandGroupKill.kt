package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.EmptyGroupException
import kotlin.collections.addAll

abstract class CommandGroupKill(vararg val commands: Command) : Command() {

    /**
     * The collection of all commands within this group.
     */
    val children: ArrayDeque<Command> = ArrayDeque(commands.toList())

    init {
        setRequirements(commands.flatMap { it.requirements }.toSet())
        if (commands.isEmpty()) throw EmptyGroupException()
    }

    override fun stop(interrupted: Boolean) {
        children.forEach { it.cancel() }
        children.clear()
        children.addAll(commands)
    }
}