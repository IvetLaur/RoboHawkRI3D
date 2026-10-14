package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.ShooterGains;

/**
 * Constants = every "magic number" and hardware name in ONE place.
 *
 * If a wire moves or a speed needs tuning, change it here and nowhere else.
 * The names in Hardware must match the Robot Configuration on the Driver Hub
 * EXACTLY (they are case-sensitive).
 */
public final class Constants {

    private Constants() {} // nobody should create a Constants object

    /** Names from the Driver Hub "Configure Robot" screen. */
    public static final class Hardware {
        public static final String FRONT_LEFT  = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_LEFT   = "backLeft";
        public static final String BACK_RIGHT  = "backRight";

        public static final String INTAKE         = "intake";
        public static final String NECTAR_SHOOTER = "nectarShooter";
        public static final String POLLEN_SHOOTER = "pollenShooter";

        public static final String IMU = "imu";             // built into the Control Hub
        public static final String LIMELIGHT = "limelight"; // optional: code runs without it
    }

    /** Mecanum drive tuning. */
    public static final class Drive {
        /** Mecanum wheels slip sideways a bit; this boosts strafe to feel equal to forward. */
        public static final double STRAFE_MULTIPLIER = 1.1;
        /** Power scale while the driver holds the slow-mode button. */
        public static final double SLOW_MODE_SCALE = 0.35;
        /** Stick values smaller than this are treated as zero (stops creeping). */
        public static final double DEADBAND = 0.05;
    }

    /** Intake tuning. */
    public static final class Intake {
        public static final double IN_POWER  = 1.0;
        public static final double OUT_POWER = -0.6; // spit pieces back out, slower
        /** How far a trigger must be pressed to count as "pressed". */
        public static final double TRIGGER_THRESHOLD = 0.2;
    }

    /** Shooter (flywheel) tuning. Get real numbers from the "Shooter Tune" OpMode. */
    public static final class Shooter {
        /**
         * Encoder ticks per FLYWHEEL revolution.
         * goBILDA 5203 bare / 6000 RPM (1:1) = 28. REV HD Hex bare = 28.
         * If there's a gearbox or belt between motor and wheel, multiply accordingly.
         */
        public static final double TICKS_PER_REV = 28.0;

        /** Voltage the gains were tuned at. Power is scaled by NOMINAL / actual battery voltage. */
        public static final double NOMINAL_VOLTAGE = 12.0;

        /** Shooter counts as "ready" when within this many RPM of target. */
        public static final double READY_TOLERANCE_RPM = 75;

        /** Integral only builds up when error is smaller than this (stops wind-up during spin-up). */
        public static final double INTEGRAL_ZONE_RPM = 300;

        /** Low-pass filter on measured RPM, 0..1. Higher = smoother but slower to react. */
        public static final double VELOCITY_FILTER = 0.3;

        /**
         * Gains + default speed for each shooter. These are STARTING GUESSES —
         * run the Shooter Tune OpMode and copy the numbers it prints back here.
         *   new ShooterGains(defaultRpm, kV, kS, kP, kI, kD)
         */
        public static final ShooterGains NECTAR = new ShooterGains(3800, 0.000170, 0.03, 0.0008, 0.0, 0.00002);
        public static final ShooterGains POLLEN = new ShooterGains(3200, 0.000170, 0.03, 0.0008, 0.0, 0.00002);

        /**
         * Distance to our upright HIVE CELL (inches) -> flywheel RPM, used by vision RPM mode.
         * PLACEHOLDERS: fill these from real test shots in Shooter Tune mode.
         * Distances must be in increasing order.
         */
        public static final double[] LUT_DISTANCE_IN = {24, 36, 48, 60, 72, 96};
        public static final double[] NECTAR_LUT_RPM  = {3000, 3300, 3550, 3800, 4050, 4500};
        public static final double[] POLLEN_LUT_RPM  = {2600, 2850, 3050, 3250, 3450, 3850};
    }

    /**
     * Limelight 3A + BIOBUZZ AprilTag CLUSTERS.
     *
     * WHAT CHANGED FROM LAST SEASON: in DECODE each goal had ONE AprilTag, so
     * "find tag 20, aim at it" worked. In BIOBUZZ every HIVE CELL has a CLUSTER of
     * FOUR tags (36h11, 3.25 in) on its underside, facing the tiles. Any one tag
     * identifies the whole CELL. The HIVE is bi-stable and FLIPS when it tips, so
     * the CELL you can score in changes during the match — the code must re-pick
     * the UPRIGHT cell of OUR alliance every loop, not remember a fixed tag ID.
     */
    public static final class Vision {
        /** Pipeline slot set up in the Limelight web UI (http://limelight.local:5801). */
        public static final int APRILTAG_PIPELINE = 0;

        /**
         * The four HIVE CELL clusters (names as in the FTC SDK 12 game library).
         * Blue audience 38-41 is confirmed in the manual text; the others come from
         * the 30-45 SDK numbering. TODO: check every ID against manual Figure 9-17.
         */
        public static final String[] CELL_NAMES = {
                "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING" };
        public static final int[][] CELL_TAG_IDS = {
                {30, 31, 32, 33},
                {34, 35, 36, 37},
                {38, 39, 40, 41},
                {42, 43, 44, 45} };

        /**
         * A CELL counts as UPRIGHT (scorable) when its cluster's |roll| is below this.
         * FIRST's tech tip uses 90 degrees with the SDK's ftcPose.roll.
         */
        public static final double UPRIGHT_MAX_ROLL_DEG = 90.0;
        /**
         * Limelight's camera-space roll may be measured from a different zero than the
         * SDK's. Point the camera at an upright cell, read "roll" on telemetry, and
         * put the number here so upright reads near 0. TODO: calibrate on the field.
         */
        public static final double ROLL_OFFSET_DEG = 0.0;

        /** Results older than this are ignored. */
        public static final long MAX_STALENESS_MS = 100;

        /** Auto-aim turn controller (power per degree of tx). */
        public static final double AIM_KP = 0.02;
        public static final double AIM_MIN_TURN = 0.06;    // overcomes friction near the target
        public static final double AIM_TOLERANCE_DEG = 1.5;
        public static final double AIM_MAX_TURN = 0.6;
    }
}
