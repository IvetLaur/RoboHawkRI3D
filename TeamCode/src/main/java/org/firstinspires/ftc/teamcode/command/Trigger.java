package org.firstinspires.ftc.teamcode.command;

import java.util.function.BooleanSupplier;

/**
 * Binds a button (or any true/false condition) to commands.
 *
 *   new Trigger(() -> gamepad2.a).toggleOnTrue(spinNectar);
 *   new Trigger(() -> gamepad2.right_trigger > 0.2).whileTrue(intakeIn);
 *
 * The scheduler checks every Trigger once per loop and compares with the
 * previous loop, so "pressed" means "just went from up to down".
 *
 * Names match WPILib (FRC) so students can read FRC docs and examples.
 */
public class Trigger {

    private final BooleanSupplier condition;

    public Trigger(BooleanSupplier condition) {
        this.condition = condition;
    }

    public boolean get() { return condition.getAsBoolean(); }

    /** Start the command when pressed. It runs until it finishes or is cancelled. */
    public Trigger onTrue(Command command) {
        addBinding((was, now) -> { if (!was && now) command.schedule(); });
        return this;
    }

    /** Start when pressed, cancel when released. */
    public Trigger whileTrue(Command command) {
        addBinding((was, now) -> {
            if (!was && now) command.schedule();
            if (was && !now) command.cancel();
        });
        return this;
    }

    /** Press once to start, press again to cancel. */
    public Trigger toggleOnTrue(Command command) {
        addBinding((was, now) -> {
            if (!was && now) {
                if (command.isScheduled()) command.cancel(); else command.schedule();
            }
        });
        return this;
    }

    /** Combine two conditions, e.g. a button that only works while another is held. */
    public Trigger and(BooleanSupplier other) {
        return new Trigger(() -> get() && other.getAsBoolean());
    }

    public Trigger negate() {
        return new Trigger(() -> !get());
    }

    // ------------------------------------------------------------ internals

    private interface Binding { void update(boolean was, boolean now); }

    private void addBinding(Binding binding) {
        final boolean[] last = { get() }; // pressed during INIT doesn't count as a new press
        CommandScheduler.getInstance().addButtonPoller(() -> {
            boolean now = get();
            binding.update(last[0], now);
            last[0] = now;
        });
    }
}
