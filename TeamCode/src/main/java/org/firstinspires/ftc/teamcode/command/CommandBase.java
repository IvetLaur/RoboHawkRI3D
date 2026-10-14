package org.firstinspires.ftc.teamcode.command;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/** Start here when writing a command: extend this and override what you need. */
public abstract class CommandBase implements Command {

    private final Set<Subsystem> requirements = new HashSet<>();

    /** "This command needs these subsystems." Call it in your constructor. */
    protected final void addRequirements(Subsystem... subsystems) {
        Collections.addAll(requirements, subsystems);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }
}
