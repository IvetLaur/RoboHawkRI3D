package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Base class for every command-based OpMode.
 *
 * You write:
 *   initialize()  create subsystems and bind buttons (runs once at INIT)
 *   initLoop()    optional: runs over and over between INIT and START
 *   loopTelemetry() optional: runs every loop after the scheduler
 *
 * This class handles the loop and always shuts every command down at STOP,
 * so motors never keep running after the match.
 */
public abstract class CommandOpMode extends LinearOpMode {

    public abstract void initialize();

    public void initLoop() {}

    public void loopTelemetry() {}

    @Override
    public final void runOpMode() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.reset(); // clear anything left from the last OpMode (e.g. Autonomous)

        try {
            initialize();

            while (opModeInInit()) {
                initLoop();
                idle();
            }

            while (opModeIsActive()) {
                scheduler.run();
                loopTelemetry();
            }
        } finally {
            scheduler.reset(); // runs end() on every command -> motors stop
        }
    }
}
