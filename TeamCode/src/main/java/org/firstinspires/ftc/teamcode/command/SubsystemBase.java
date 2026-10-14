package org.firstinspires.ftc.teamcode.command;

/**
 * Extend this for every subsystem. It registers itself with the scheduler
 * automatically, so its periodic() runs every loop.
 */
public abstract class SubsystemBase implements Subsystem {

    private String name = getClass().getSimpleName();

    protected SubsystemBase() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public String getName() { return name; }

    public void setName(String name) { this.name = name; }
}
