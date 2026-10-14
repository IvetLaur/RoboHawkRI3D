package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.InstantCommand;
import org.firstinspires.ftc.teamcode.command.Trigger;
import org.firstinspires.ftc.teamcode.commands.AimAtCellCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.VisionRpmShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * ROBOHAWK RI3D — BIOBUZZ COMPETITION TELEOP.
 *
 * In command-based code the OpMode is short. It only does three jobs:
 *   1. CREATE the subsystems (the robot's parts)
 *   2. BIND buttons to commands (what each button does)
 *   3. Let the CommandScheduler run everything (CommandOpMode does this for us)
 *
 * DURING INIT: Gamepad 1 d-pad LEFT = BLUE alliance, d-pad RIGHT = RED alliance.
 *
 * CONTROLS
 *  Gamepad 1 (driver)
 *    Left stick ........ drive forward/back + strafe
 *    Right stick X ..... turn
 *    Right bumper ...... hold for slow mode
 *    Left bumper ....... hold to AUTO-AIM at our upright HIVE CELL
 *    Y ................. toggle field-centric / robot-centric
 *    Back .............. reset heading (point robot away from you first)
 *  Gamepad 2 (operator)
 *    Right trigger ..... intake in (hold)
 *    Left trigger ...... intake out (hold)
 *    A ................. NECTAR shooter on/off (default speed)
 *    B ................. POLLEN shooter on/off (default speed)
 *    Right bumper ...... NECTAR shooter on/off, speed from distance to cell
 *    Left bumper ....... POLLEN shooter on/off, speed from distance to cell
 *    X ................. stop BOTH shooters
 */
@TeleOp(name = "RoboHawk TeleOp", group = "RoboHawk")
public class MainTeleOp extends CommandOpMode {

    private Alliance alliance = Alliance.BLUE;

    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem nectarShooter;
    private ShooterSubsystem pollenShooter;
    private VisionSubsystem vision;

    @Override
    public void initialize() {
        // ---------- 1. Subsystems (each registers itself with the scheduler) ----------
        drive = new MecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        nectarShooter = new ShooterSubsystem(hardwareMap, Constants.Hardware.NECTAR_SHOOTER,
                Constants.Shooter.NECTAR, false);
        pollenShooter = new ShooterSubsystem(hardwareMap, Constants.Hardware.POLLEN_SHOOTER,
                Constants.Shooter.POLLEN, false);
        vision = new VisionSubsystem(hardwareMap);

        // ---------- 2a. Default command: driving ----------
        // The FTC gamepad reports stick UP as negative, so we flip Y.
        drive.setDefaultCommand(new DriveCommand(
                drive,
                () -> -gamepad1.left_stick_y,  // forward
                () -> gamepad1.left_stick_x,   // strafe
                () -> gamepad1.right_stick_x,  // turn
                () -> gamepad1.right_bumper    // slow mode
        ));

        // ---------- 2b. Driver buttons ----------
        new Trigger(() -> gamepad1.y).onTrue(new InstantCommand(drive::toggleFieldCentric));
        new Trigger(() -> gamepad1.back).onTrue(new InstantCommand(drive::resetHeading));

        new Trigger(() -> gamepad1.left_bumper).whileTrue(new AimAtCellCommand(
                drive, vision,
                () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x,
                () -> alliance));

        // ---------- 2c. Intake (triggers are analog, so compare to a threshold) ----------
        new Trigger(() -> gamepad2.right_trigger > Constants.Intake.TRIGGER_THRESHOLD)
                .whileTrue(new IntakeCommand(intake, IntakeCommand.Direction.IN));
        new Trigger(() -> gamepad2.left_trigger > Constants.Intake.TRIGGER_THRESHOLD)
                .whileTrue(new IntakeCommand(intake, IntakeCommand.Direction.OUT));

        // ---------- 2d. Shooters ----------
        SpinShooterCommand spinNectar = new SpinShooterCommand(nectarShooter);
        SpinShooterCommand spinPollen = new SpinShooterCommand(pollenShooter);
        VisionRpmShooterCommand visionNectar = new VisionRpmShooterCommand(nectarShooter, vision,
                () -> alliance, Constants.Shooter.LUT_DISTANCE_IN, Constants.Shooter.NECTAR_LUT_RPM);
        VisionRpmShooterCommand visionPollen = new VisionRpmShooterCommand(pollenShooter, vision,
                () -> alliance, Constants.Shooter.LUT_DISTANCE_IN, Constants.Shooter.POLLEN_LUT_RPM);

        new Trigger(() -> gamepad2.a).toggleOnTrue(spinNectar);
        new Trigger(() -> gamepad2.b).toggleOnTrue(spinPollen);
        new Trigger(() -> gamepad2.right_bumper).toggleOnTrue(visionNectar);
        new Trigger(() -> gamepad2.left_bumper).toggleOnTrue(visionPollen);

        // X = stop both flywheels. Cancelling a command runs its end(), which stops the motor.
        new Trigger(() -> gamepad2.x).onTrue(new InstantCommand(() -> {
            spinNectar.cancel();
            spinPollen.cancel();
            visionNectar.cancel();
            visionPollen.cancel();
        }));
    }

    /** Between INIT and START: pick alliance and check the camera sees the HIVE. */
    @Override
    public void initLoop() {
        if (gamepad1.dpad_left) alliance = Alliance.BLUE;
        if (gamepad1.dpad_right) alliance = Alliance.RED;

        vision.periodic(); // the scheduler isn't running yet, so update the camera by hand
        telemetry.addData("Alliance", "%s   (gp1 d-pad left = BLUE, right = RED)", alliance);
        addVisionTelemetry();
        telemetry.addLine("Ready. Press START.");
        telemetry.update();
    }

    @Override
    public void loopTelemetry() {
        telemetry.addData("Drive", "%s  heading %.1f°",
                drive.isFieldCentric() ? "FIELD-centric" : "ROBOT-centric",
                Math.toDegrees(drive.getHeadingRadians()));
        telemetry.addData("Intake power", "%.2f", intake.getPower());
        addShooterTelemetry("Nectar", nectarShooter);
        addShooterTelemetry("Pollen", pollenShooter);
        telemetry.addData("Battery", "%.2f V", nectarShooter.getBatteryVoltage());
        addVisionTelemetry();
        telemetry.update();
    }

    private void addVisionTelemetry() {
        if (!vision.isConnected()) {
            telemetry.addData("Limelight", "NOT configured");
            return;
        }
        VisionSubsystem.HiveCell target = vision.getTargetCell(alliance);
        telemetry.addData("Target cell", target == null ? "none (" + alliance + ", upright)"
                : String.format("%s  tx %.1f°  %s", target.name, target.txDeg,
                        Double.isNaN(target.distanceIn) ? "dist -"
                                : String.format("%.0f in", target.distanceIn)));
        for (VisionSubsystem.HiveCell c : vision.getVisibleCells()) {
            telemetry.addLine("  " + c);
        }
        telemetry.addData("Raw tag IDs", vision.getRawTagIds());
    }

    private void addShooterTelemetry(String label, ShooterSubsystem shooter) {
        String status = !shooter.isRunning() ? "off"
                : shooter.isAtSpeed() ? "READY" : "spinning up";
        telemetry.addData(label, "%s  %.0f / %.0f rpm",
                status, shooter.getCurrentRpm(), shooter.getTargetRpm());
    }
}
