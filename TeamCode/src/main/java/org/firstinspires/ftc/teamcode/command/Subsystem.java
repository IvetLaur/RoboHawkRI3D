package org.firstinspires.ftc.teamcode.command;

/**
 * A SUBSYSTEM is one part of the robot that owns its hardware
 * (drivetrain, intake, a shooter, the camera).
 *
 * Only the subsystem touches its motors. Commands ask it to do things.
 */
public interface Subsystem {

    /** Runs every loop, before commands. Good for control loops and reading sensors. */
    default void periodic() {}

    /** The command this subsystem runs whenever nothing else is using it. */
    default void setDefaultCommand(Command command) {
        CommandScheduler.getInstance().setDefaultCommand(this, command);
    }
}
