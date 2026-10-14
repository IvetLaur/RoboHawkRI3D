package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.InstantCommand;
import org.firstinspires.ftc.teamcode.command.Trigger;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGains;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * SHOOTER TUNE MODE — tune the flywheel PIDF live, from the Driver Hub.
 *
 * Everything on GAMEPAD 1:
 *   Sticks ........... drive (to test shots from different distances)
 *   D-pad left/right . pick NECTAR or POLLEN shooter
 *   A ................ selected shooter on/off
 *   Y ................ next setting   (RPM -> kV -> kS -> kP -> kI -> kD)
 *   D-pad up/down .... change the setting by one step
 *   X ................ step size: x1 -> x10 -> x0.1
 *   Right trigger .... run intake to feed a shot (watch the recovery numbers)
 *   Back ............. clear shot stats
 *
 * TUNING RECIPE (do it in this order):
 *   1. Set kP, kI, kD to 0. Spin up.
 *   2. Raise/lower kV until "actual" sits just under "target" by itself.
 *      Tip: the "kV guess" line does this math for you once the wheel is steady.
 *   3. Add kS only if the wheel struggles at low RPM (usually 0.02–0.05).
 *   4. Raise kP until a shot's "recovery" time is short without the RPM
 *      bouncing above target. If it oscillates, back off ~30%.
 *   5. Only if there's a small steady error left: tiny kI. kD rarely needed.
 *   6. Copy the "Paste into Constants" line into Constants.java. Changes made here
 *      are LOST when the robot restarts!
 *
 * BUILDING THE DISTANCE TABLE: point at an upright HIVE CELL so its cluster is
 * in view, drive to a spot, adjust RPM until shots land in the CELL, write down
 * (distance, RPM). Repeat at 5–6 distances and put them in Constants.Shooter.*_LUT_*.
 * The distance shown is to ANY upright cell (either alliance) so you can test
 * from both sides.
 */
@TeleOp(name = "RoboHawk Shooter Tune", group = "RoboHawk")
public class ShooterTuneOpMode extends CommandOpMode {

    private enum Param { RPM, KV, KS, KP, KI, KD }

    private static final double[] STEP_MULTIPLIERS = {1, 10, 0.1};

    private ShooterSubsystem nectar, pollen, selected;
    private IntakeSubsystem intake;
    private MecanumDriveSubsystem drive;
    private VisionSubsystem vision;

    private Param param = Param.RPM;
    private int stepIndex = 0;

    // Shot-recovery measurement
    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean wasAtSpeed = false, inDip = false;
    private double dipMinRpm = 0, lastDipRpm = Double.NaN, lastRecoveryMs = Double.NaN;
    private int shots = 0;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        nectar = new ShooterSubsystem(hardwareMap, Constants.Hardware.NECTAR_SHOOTER, Constants.Shooter.NECTAR, false);
        pollen = new ShooterSubsystem(hardwareMap, Constants.Hardware.POLLEN_SHOOTER, Constants.Shooter.POLLEN, false);
        vision = new VisionSubsystem(hardwareMap);
        selected = nectar;

        drive.setDefaultCommand(new DriveCommand(drive,
                () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x,
                () -> true)); // always slow mode while tuning — safer

        // Same Trigger bindings as the competition TeleOp — just different actions.
        new Trigger(() -> gamepad1.dpad_left).onTrue(new InstantCommand(() -> select(nectar)));
        new Trigger(() -> gamepad1.dpad_right).onTrue(new InstantCommand(() -> select(pollen)));
        new Trigger(() -> gamepad1.a).onTrue(new InstantCommand(() -> {
            if (selected.isRunning()) selected.stop(); else selected.spinUp();
        }));
        new Trigger(() -> gamepad1.y).onTrue(new InstantCommand(() ->
                param = Param.values()[(param.ordinal() + 1) % Param.values().length]));
        new Trigger(() -> gamepad1.x).onTrue(new InstantCommand(() ->
                stepIndex = (stepIndex + 1) % STEP_MULTIPLIERS.length));
        new Trigger(() -> gamepad1.dpad_up).onTrue(new InstantCommand(() -> adjust(+1)));
        new Trigger(() -> gamepad1.dpad_down).onTrue(new InstantCommand(() -> adjust(-1)));
        new Trigger(() -> gamepad1.back).onTrue(new InstantCommand(() -> {
            shots = 0;
            lastDipRpm = Double.NaN;
            lastRecoveryMs = Double.NaN;
        }));
        new Trigger(() -> gamepad1.right_trigger > Constants.Intake.TRIGGER_THRESHOLD)
                .whileTrue(new IntakeCommand(intake, IntakeCommand.Direction.IN));

        telemetry.addLine("Shooter Tune ready. Press START.");
        telemetry.update();
    }

    /** After the scheduler ran this loop (so the PIDF just updated): stats + screen. */
    @Override
    public void loopTelemetry() {
        trackShotRecovery();
        showTelemetry();
    }

    private void select(ShooterSubsystem s) {
        if (s == selected) return;
        selected.stop(); // only one wheel at a time while tuning
        selected = s;
        wasAtSpeed = inDip = false;
    }

    private void adjust(int direction) {
        ShooterGains g = selected.getGains();
        double step = baseStep(param) * STEP_MULTIPLIERS[stepIndex] * direction;
        switch (param) {
            case RPM: g.defaultRpm = Math.max(0, g.defaultRpm + step); break;
            case KV:  g.kV = Math.max(0, g.kV + step); break;
            case KS:  g.kS = Math.max(0, g.kS + step); break;
            case KP:  g.kP = Math.max(0, g.kP + step); break;
            case KI:  g.kI = Math.max(0, g.kI + step); break;
            case KD:  g.kD = Math.max(0, g.kD + step); break;
        }
        // Gains are read live every loop; RPM needs to be re-applied.
        if (param == Param.RPM && selected.isRunning()) selected.spinUp();
    }

    private static double baseStep(Param p) {
        switch (p) {
            case RPM: return 100;
            case KV:  return 0.000005;
            case KS:  return 0.005;
            case KP:  return 0.0001;
            case KI:  return 0.00001;
            default:  return 0.000005; // KD
        }
    }

    // ------------------------------------------------------------------ shot stats

    /** A "shot" = wheel was at speed, then dropped below. Measure dip and time to recover. */
    private void trackShotRecovery() {
        boolean atSpeed = selected.isAtSpeed();
        double rpm = selected.getCurrentRpm();

        if (wasAtSpeed && !atSpeed && selected.isRunning() && selected.getError() > 0) {
            inDip = true;
            dipMinRpm = rpm;
            shotTimer.reset();
            shots++;
        }
        if (inDip) {
            dipMinRpm = Math.min(dipMinRpm, rpm);
            if (atSpeed) {
                inDip = false;
                lastDipRpm = selected.getTargetRpm() - dipMinRpm;
                lastRecoveryMs = shotTimer.milliseconds();
            }
        }
        wasAtSpeed = atSpeed;
    }

    // ------------------------------------------------------------------ telemetry

    private void showTelemetry() {
        ShooterGains g = selected.getGains();
        String name = selected == nectar ? "NECTAR" : "POLLEN";

        telemetry.addData("Shooter", "%s  %s", name,
                !selected.isRunning() ? "off" : selected.isAtSpeed() ? "READY" : "spinning up");
        telemetry.addData("RPM", "target %.0f   actual %.0f   error %+.0f",
                selected.getTargetRpm(), selected.getCurrentRpm(), selected.getError());
        telemetry.addData("Power / Battery", "%.3f   %.2f V", selected.getPower(), selected.getBatteryVoltage());
        telemetry.addLine();

        telemetry.addData("Editing", "%s   step x%s", param, fmtMult(STEP_MULTIPLIERS[stepIndex]));
        telemetry.addLine(row(Param.RPM, "RPM", String.format("%.0f", g.defaultRpm)));
        telemetry.addLine(row(Param.KV, "kV ", String.format("%.6f", g.kV)));
        telemetry.addLine(row(Param.KS, "kS ", String.format("%.3f", g.kS)));
        telemetry.addLine(row(Param.KP, "kP ", String.format("%.6f", g.kP)));
        telemetry.addLine(row(Param.KI, "kI ", String.format("%.6f", g.kI)));
        telemetry.addLine(row(Param.KD, "kD ", String.format("%.6f", g.kD)));
        telemetry.addLine();

        // kV guess: power that's holding steady speed, converted back to 12 V, per RPM.
        if (selected.isAtSpeed() && selected.getCurrentRpm() > 500) {
            double kvGuess = (selected.getPower() * selected.getBatteryVoltage()
                    / Constants.Shooter.NOMINAL_VOLTAGE - g.kS) / selected.getCurrentRpm();
            telemetry.addData("kV guess (set kP=0 first)", "%.6f", kvGuess);
        }
        telemetry.addData("Shots", "%d   last dip %s rpm   recovery %s ms", shots,
                Double.isNaN(lastDipRpm) ? "-" : String.format("%.0f", lastDipRpm),
                Double.isNaN(lastRecoveryMs) ? "-" : String.format("%.0f", lastRecoveryMs));

        if (vision.isConnected()) {
            VisionSubsystem.HiveCell cell = vision.getTargetCell(Alliance.RED);
            if (cell == null) cell = vision.getTargetCell(Alliance.BLUE);
            telemetry.addData("Upright cell", cell == null ? "none in view"
                    : String.format("%s  %s", cell.name, Double.isNaN(cell.distanceIn)
                            ? "dist - (set tag size in pipeline)"
                            : String.format("%.0f in", cell.distanceIn)));
        }
        telemetry.addLine();
        telemetry.addLine("Paste into Constants." + name + ":");
        telemetry.addLine(g.toString());
        telemetry.update();
    }

    private String row(Param p, String label, String value) {
        return (param == p ? "▶ " : "   ") + label + "  " + value;
    }

    private static String fmtMult(double m) {
        return m == 0.1 ? "0.1" : String.valueOf((int) m);
    }
}
