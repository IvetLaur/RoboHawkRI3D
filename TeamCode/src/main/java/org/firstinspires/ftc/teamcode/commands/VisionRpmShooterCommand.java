package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LookupTable;

import java.util.function.Supplier;

/**
 * COMMAND: spin a shooter at the speed that matches how far away our upright
 * HIVE CELL is.
 *
 * Every loop: distance to the target cell -> lookup table -> RPM.
 * The distance is the AVERAGE of every visible tag in the cell's cluster, so it
 * jumps around less than a single tag did last season.
 *
 * Lost the cell? Keep the last good speed. Never saw it? Shooter's default speed.
 * Requires the shooter, so turning this on replaces the manual SpinShooterCommand.
 */
public class VisionRpmShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final Supplier<Alliance> alliance;
    private final LookupTable table;

    public VisionRpmShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision,
                                   Supplier<Alliance> alliance,
                                   double[] distancesIn, double[] rpms) {
        this.shooter = shooter;
        this.vision = vision;
        this.alliance = alliance;
        this.table = new LookupTable(distancesIn, rpms);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.spinUp(); // default speed until we see the cell
    }

    @Override
    public void execute() {
        VisionSubsystem.HiveCell cell = vision.getTargetCell(alliance.get());
        if (cell != null && !Double.isNaN(cell.distanceIn)) {
            shooter.setTargetRpm(table.get(cell.distanceIn));
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
