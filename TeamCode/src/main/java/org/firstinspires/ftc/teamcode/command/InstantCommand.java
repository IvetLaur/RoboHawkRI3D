package org.firstinspires.ftc.teamcode.command;

/**
 * A command that does one thing once and is immediately finished.
 * Example: new InstantCommand(drive::resetHeading)
 */
public class InstantCommand extends CommandBase {

    private final Runnable action;

    public InstantCommand(Runnable action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void initialize() { action.run(); }

    @Override
    public boolean isFinished() { return true; }
}
