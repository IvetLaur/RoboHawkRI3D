package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * COMMAND: run the intake in one direction for as long as the command is active.
 *
 * Bound with Trigger.whileTrue(), so it starts when the trigger is
 * pressed and end() stops the roller the moment it's released.
 */
public class IntakeCommand extends CommandBase {

    public enum Direction { IN, OUT }

    private final IntakeSubsystem intake;
    private final Direction direction;

    public IntakeCommand(IntakeSubsystem intake, Direction direction) {
        this.intake = intake;
        this.direction = direction;
        addRequirements(intake);
    }

    /** Called once, when the command starts. */
    @Override
    public void initialize() {
        if (direction == Direction.IN) {
            intake.intake();
        } else {
            intake.outtake();
        }
    }

    /** Called once, when the command stops for any reason. */
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
