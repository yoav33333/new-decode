package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.CommandGroup
import dev.nextftc.core.commands.groups.SequentialGroup

class CommandSeqNoReq(vararg commands: Command) : CommandGroupNoReq(*commands) {
    init {
        named("CommandSeqNoReq(${children.joinToString { it.name }})")
    }
    /**
     * This returns true once all of its children have finished running.
     */
    override val isDone: Boolean
        get() = children.isEmpty()

    /**
     * In a Sequential Group, we will start the first command and wait until it has completed
     * execution before starting the next.
     */
    override fun start() {
        children.first().start()
    }

    /**
     * Now, every update we must check if the currently active command is complete. If it is, remove
     * it and start the next one (if there is one).
     */
    override fun update() {
        children.first().update()

        if (!children.first().isDone) return

        children.removeFirst().stop(false)

        if (children.isNotEmpty()) children.first().start()
    }

    override fun stop(interrupted: Boolean) {
        if (children.isNotEmpty()) children.first().stop(interrupted)

        super.stop(interrupted)
    }

    override fun then(vararg commands: Command): SequentialGroup =
        SequentialGroup(*children.toTypedArray(), *commands)
}