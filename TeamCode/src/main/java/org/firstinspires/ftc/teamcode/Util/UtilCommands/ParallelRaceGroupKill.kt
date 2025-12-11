package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup

class ParallelRaceGroupKill(vararg commands: Command) : ParallelGroup(*commands) {
    init {
        named("ParallelRaceGroup(${children.joinToString { it.name }})")
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
     * This will return false until one of its children is done
     */
    override val isDone: Boolean
        get() = children.any { it.isDone }
}