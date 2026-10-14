package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * COMMAND: drive with the joysticks.
 *
 * This is the drivetrain's DEFAULT command — the scheduler runs it any time no
 * other command is using the drivetrain (in TeleOp, that's always).
 *
 * Why "Suppliers"? A DoubleSupplier is a tiny function that READS the stick
 * each time we call getAsDouble(). If we passed in a plain double, we'd only
 * get the stick value from the moment the command was created — forever.
 */
public class DriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final DoubleSupplier forward, strafe, turn;
    private final BooleanSupplier slowMode;

    public DriveCommand(MecanumDriveSubsystem drive,
                        DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn,
                        BooleanSupplier slowMode) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.slowMode = slowMode;
        // "I need the drivetrain." The scheduler uses this to make sure only
        // one command controls the drivetrain at a time.
        addRequirements(drive);
    }

    /** Called over and over (~every loop) while the command is scheduled. */
    @Override
    public void execute() {
        double scale = slowMode.getAsBoolean() ? Constants.Drive.SLOW_MODE_SCALE : 1.0;
        drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble(), scale);
    }

    /** Called once when the command stops (OpMode ends, or interrupted). */
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // isFinished() is not overridden, so it returns false: driving never "finishes".
}
