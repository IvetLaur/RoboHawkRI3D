package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * COMMAND: keep a shooter flywheel spinning until the command is cancelled.
 *
 * Works for BOTH the nectar and the pollen shooter — you pass in which one.
 * In TeleOp it's bound with Trigger.toggleOnTrue(): press once = spin up,
 * press again = command cancelled = end() = flywheel stops.
 */
public class SpinShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;

    public SpinShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.spinUp();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    // isFinished() stays false: it runs until someone cancels it.
}
