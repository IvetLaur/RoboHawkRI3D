package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * COMMAND: driver keeps moving with the left stick, the ROBOT turns to face
 * our alliance's UPRIGHT HIVE CELL.
 *
 * Because the HIVE flips when it tips, the target is looked up again every
 * loop — if the other cell becomes upright mid-match, the robot swings to it.
 *
 * Bound with whileTrue: holding the button takes over the drivetrain (the normal
 * DriveCommand pauses), letting go hands it back automatically.
 * No cell visible? The driver's own turn stick still works.
 */
public class AimAtCellCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final VisionSubsystem vision;
    private final DoubleSupplier forward, strafe, manualTurn;
    private final Supplier<Alliance> alliance;

    public AimAtCellCommand(MecanumDriveSubsystem drive, VisionSubsystem vision,
                            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier manualTurn,
                            Supplier<Alliance> alliance) {
        this.drive = drive;
        this.vision = vision;
        this.forward = forward;
        this.strafe = strafe;
        this.manualTurn = manualTurn;
        this.alliance = alliance;
        // Only the drivetrain: we READ the camera but don't control it.
        addRequirements(drive);
    }

    @Override
    public void execute() {
        VisionSubsystem.HiveCell cell = vision.getTargetCell(alliance.get());
        double turn;

        if (cell == null) {
            turn = manualTurn.getAsDouble();
        } else if (Math.abs(cell.txDeg) < Constants.Vision.AIM_TOLERANCE_DEG) {
            turn = 0; // close enough — stop wiggling
        } else {
            // P-controller: further off = turn harder, plus a minimum to beat friction.
            turn = cell.txDeg * Constants.Vision.AIM_KP
                    + Math.signum(cell.txDeg) * Constants.Vision.AIM_MIN_TURN;
            turn = Math.max(-Constants.Vision.AIM_MAX_TURN, Math.min(Constants.Vision.AIM_MAX_TURN, turn));
        }

        drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn, 1.0);
    }

    /** Useful later in Autonomous: "aim, then shoot when lined up". */
    public boolean isAimed() {
        VisionSubsystem.HiveCell cell = vision.getTargetCell(alliance.get());
        return cell != null && Math.abs(cell.txDeg) < Constants.Vision.AIM_TOLERANCE_DEG;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
