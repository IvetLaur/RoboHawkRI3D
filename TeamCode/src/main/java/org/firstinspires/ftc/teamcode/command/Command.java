package org.firstinspires.ftc.teamcode.command;

import java.util.Set;

/**
 * A COMMAND is one robot action ("spin the nectar shooter", "drive with sticks").
 *
 * Life cycle, run by the CommandScheduler:
 *   initialize()      once, when the command starts
 *   execute()         every loop while it runs
 *   isFinished()      every loop — return true to stop by itself
 *   end(interrupted)  once, when it stops (finished, cancelled, or replaced)
 *
 * This is our own small version of the WPILib-style command pattern that FRC
 * and many FTC teams use — no outside library needed.
 */
public interface Command {

    default void initialize() {}

    default void execute() {}

    default boolean isFinished() { return false; }

    /** @param interrupted true if cancelled or replaced, false if isFinished() said so */
    default void end(boolean interrupted) {}

    /** The subsystems this command controls. Two commands can't share one at the same time. */
    Set<Subsystem> getRequirements();

    default void schedule() { CommandScheduler.getInstance().schedule(this); }

    default void cancel() { CommandScheduler.getInstance().cancel(this); }

    default boolean isScheduled() { return CommandScheduler.getInstance().isScheduled(this); }
}
