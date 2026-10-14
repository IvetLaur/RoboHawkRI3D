package org.firstinspires.ftc.teamcode.command;

/**
 * A command that runs the same code every loop until cancelled.
 * Example: new RunCommand(() -> intake.intake(), intake)
 */
public class RunCommand extends CommandBase {

    private final Runnable action;

    public RunCommand(Runnable action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void execute() { action.run(); }
}
