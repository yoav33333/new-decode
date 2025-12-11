package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.EmptyGroupException
import dev.nextftc.core.commands.groups.ParallelGroup
import kotlin.collections.addAll

class ParallelDeadlineGroupKill(private val deadline: Command, vararg otherCommands: Command) :
    ParallelGroup(deadline, *otherCommands) {

    init {
        named("ParallelDeadlineGroup(${deadline.name} | ${otherCommands.joinToString { it.name }})")
    }

    override fun stop(interrupted: Boolean) {
        children.forEach {
            it.stop(interrupted)
        }
        children.forEach { it.cancel() }
        children.clear()
        children.addAll(commands)
    }
    /**
     * This will return false until the deadline command is done.
     */

    override val isDone: Boolean by deadline::isDone
}