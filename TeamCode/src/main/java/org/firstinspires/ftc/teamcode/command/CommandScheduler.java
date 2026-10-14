package org.firstinspires.ftc.teamcode.command;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * THE SCHEDULER — the "head chef". One per robot (a singleton).
 *
 * Every call to run() does, in order:
 *   1. Check every button binding (Triggers) — may start or cancel commands
 *   2. Call periodic() on every subsystem
 *   3. Call execute() on every running command; end the ones that finished
 *   4. Start the default command of any subsystem nobody is using
 *
 * Its most important rule: ONE command per subsystem at a time. If a new
 * command needs a subsystem that's busy, the old command is ended first.
 */
public final class CommandScheduler {

    private static CommandScheduler instance;

    public static CommandScheduler getInstance() {
        if (instance == null) instance = new CommandScheduler();
        return instance;
    }

    private CommandScheduler() {}

    private final Set<Subsystem> subsystems = new LinkedHashSet<>();
    private final Map<Subsystem, Command> defaultCommands = new HashMap<>();
    private final Set<Command> scheduled = new LinkedHashSet<>();
    private final Map<Subsystem, Command> owners = new HashMap<>();
    private final List<Runnable> buttonPollers = new ArrayList<>();

    // ------------------------------------------------------------ setup

    public void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void setDefaultCommand(Subsystem subsystem, Command command) {
        if (!command.getRequirements().contains(subsystem)) {
            throw new IllegalArgumentException("A default command must require its subsystem");
        }
        defaultCommands.put(subsystem, command);
    }

    /** Used by Trigger to check its button every loop. */
    void addButtonPoller(Runnable poller) {
        buttonPollers.add(poller);
    }

    // ------------------------------------------------------------ scheduling

    public void schedule(Command command) {
        if (scheduled.contains(command)) return;

        // Free up every subsystem this command needs.
        for (Subsystem s : command.getRequirements()) {
            Command current = owners.get(s);
            if (current != null) cancel(current);
        }

        scheduled.add(command);
        for (Subsystem s : command.getRequirements()) owners.put(s, command);
        command.initialize();
    }

    public void cancel(Command command) {
        if (!scheduled.contains(command)) return;
        finish(command, true);
    }

    public void cancelAll() {
        for (Command c : new ArrayList<>(scheduled)) cancel(c);
    }

    public boolean isScheduled(Command command) {
        return scheduled.contains(command);
    }

    private void finish(Command command, boolean interrupted) {
        scheduled.remove(command);
        for (Subsystem s : command.getRequirements()) {
            if (owners.get(s) == command) owners.remove(s);
        }
        command.end(interrupted);
    }

    // ------------------------------------------------------------ the loop

    public void run() {
        // 1. Buttons
        for (Runnable poller : new ArrayList<>(buttonPollers)) poller.run();

        // 2. Subsystems
        for (Subsystem s : subsystems) s.periodic();

        // 3. Commands (copy the set: commands may start/stop others while running)
        for (Command c : new ArrayList<>(scheduled)) {
            if (!scheduled.contains(c)) continue; // cancelled by someone earlier this loop
            c.execute();
            if (c.isFinished()) finish(c, false);
        }

        // 4. Default commands for idle subsystems
        for (Map.Entry<Subsystem, Command> e : defaultCommands.entrySet()) {
            if (!owners.containsKey(e.getKey())) schedule(e.getValue());
        }
    }

    /** Ends every command and forgets everything. Called at the start and end of each OpMode. */
    public void reset() {
        cancelAll();
        subsystems.clear();
        defaultCommands.clear();
        owners.clear();
        buttonPollers.clear();
    }
}
