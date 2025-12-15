package org.firstinspires.ftc.teamcode.Util.UtilCommands

import dev.nextftc.core.commands.Command
import java.util.function.BooleanSupplier


/**
 * A command that runs a given command and, if a condition is met,
 * retries it (or a different command) up to a specified number of times.
 *
 *
 * This command is useful for actions that may not succeed on the first attempt
 * and require re-running, such as vision alignment or precise mechanism movement.
 */
class LoopingCommand(
    command: Command,
    retryCommand: Command,
    successCondition: BooleanSupplier,
    maxRetries: Int
) : Command() {
    private val command: Command?
    private val retryCommand: Command?
    private val successCondition: BooleanSupplier
    private val maxRetries: Int

    private var currentCommand: Command? = null
    private var retryCount = 0
    override var isDone: Boolean = false
        private set

    /**
     * Creates a new RetryCommand.
     *
     * @param command        Supplies the command to run on the first attempt.
     * @param retryCommand   A function that takes the retry count (starting at 1) and returns the command for that attempt.
     * @param successCondition A condition that returns `true` if a retry should be attempted, or `false` if the command should finish without retrying.
     * @param maxRetries     The maximum number of retries allowed.
     */
    init {
        this.command = command
        this.retryCommand = retryCommand
        this.successCondition = successCondition
        this.maxRetries = maxRetries

        addRequirements(command.requirements)
        addRequirements(retryCommand.requirements)
    }

    /**
     * Creates a new RetryCommand where the retry command is the same as the initial one.
     *
     * @param command        A supplier that creates a new instance of the command to run.
     * @param successCondition A condition that returns `true` if a retry should be attempted, or `false` if the command should finish without retrying.
     * @param maxRetries     The maximum number of retries allowed.
     */
    constructor(
        command: Command,
        successCondition: BooleanSupplier,
        maxRetries: Int
    ) : this(command, command, successCondition, maxRetries)

    /**
     * Creates a new RetryCommand where the retry command is the same as the initial one.
     *
     * @param command        A supplier that creates a new instance of the command to run.
     * @param successCondition A condition that returns `true` if a retry should be attempted, or `false` if the command should finish without retrying.
     */
    constructor(
        command: Command,
        successCondition: BooleanSupplier,
    ) : this(command, command, successCondition, -1)

    public override fun start() {
        this.isDone = false
        retryCount = 0

        currentCommand = command
        currentCommand!!.start()
    }

    public override fun update() {
        // If the sub-command is not finished and still running (not interrupted)
        currentCommand?.isDone?.let {
            if (!it) {
                currentCommand!!.update()
                return
            }
        }

        currentCommand!!.stop(false)

        // Check if we should retry
        if (retryCount < maxRetries && !successCondition.getAsBoolean()) {
            retryCount++
            currentCommand = retryCommand
            currentCommand!!.start()
        } else {
            this.isDone = true
        }
    }

    public override fun stop(interrupted: Boolean) {
        // When RetryCommand is ended (for any reason), we must also end the sub-command it is currently managing
        if (currentCommand != null) {
            currentCommand!!.stop(interrupted)
        }
    }
}