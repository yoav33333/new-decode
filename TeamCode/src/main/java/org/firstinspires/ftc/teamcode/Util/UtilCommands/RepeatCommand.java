package org.firstinspires.ftc.teamcode.Util.UtilCommands;

import java.util.function.BooleanSupplier;

import dev.nextftc.core.commands.Command;

public class RepeatCommand extends Command {

    protected final Command m_command;
    private int timesRepeated;
    private int maxRepeatTimes = 0; // 0 implies infinite unless condition or limit is set
    private BooleanSupplier condition;

    /**
     * Creates a new RepeatCommand. Will run another command repeatedly, restarting it whenever it
     * ends, until this command is interrupted.
     *
     * @param command the command to run repeatedly
     */
    public RepeatCommand(Command command) {
        m_command = command;
        setRequirements(command.getRequirements());
    }

    /**
     * Creates a new overloaded RepeatCommand. Will run another command repeatedly, restarting it whenever it
     * ends, until this command is interrupted or a condition is met. Effectively acts as a repeat until.
     *
     * @param command   the command to run repeatedly
     * @param condition the condition to end the command
     */
    public RepeatCommand(Command command, BooleanSupplier condition) {
        this(command);
        this.condition = condition;
    }

    /**
     * Creates a new overloaded RepeatCommand. Runs another command maxRepeatTimes amount of times, and ends when
     * it has repeated enough times or if this command is interrupted.
     *
     * @param command        the command to run repeatedly
     * @param maxRepeatTimes the number of times to repeat the command (has to be greater than 0)
     */
    public RepeatCommand(Command command, int maxRepeatTimes) {
        this(command);
        if (maxRepeatTimes <= 0) {
            throw new IllegalArgumentException("RepeatCommands' maxRepeatTimes cannot be negative or zero!");
        }
        this.maxRepeatTimes = maxRepeatTimes;
        this.timesRepeated = 0;
    }

    @Override
    public void start() {
        // Reset counter if we are using count-based repetition
        if (maxRepeatTimes > 0) {
            timesRepeated = 0;
        }
        m_command.start();
    }

    @Override
    public void update() {
        m_command.update();

        if (m_command.isDone()) {
            // Count completion if we are tracking repetitions
            if (maxRepeatTimes > 0) {
                timesRepeated++;
            }

            // If we aren't finished yet (checked by getIsDone below), restart the inner command
            if (!isDone()) {
                m_command.stop(false);
                m_command.start();
            }
        }
    }

    public boolean isDone() {
        // Check repeat count limit
        if (maxRepeatTimes > 0 && timesRepeated >= maxRepeatTimes) {
            return true;
        }
        // Check external condition
        if (condition != null && condition.getAsBoolean()) {
            return true;
        }
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        // Ensure the inner command is stopped when this wrapper stops
        m_command.stop(interrupted);
    }
}
